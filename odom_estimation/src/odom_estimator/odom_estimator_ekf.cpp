#include "odom_estimation/odom_estimator/odom_estimator_ekf.hpp"
#include "kinematics/kinematics.hpp"

namespace gole {
namespace odom_estimation {

OdomEstimatorEKF::OdomEstimatorEKF(rclcpp_lifecycle::LifecycleNode* node,
                           std::shared_ptr<spdlog::logger> logger,
                           std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager,
                           const Params& params)
    : OdomEstimatorBase(node, logger, diagnostic_manager, params)
{
    sensor_fusion_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&OdomEstimatorEKF::processSensorFusionTick, this));
}

void OdomEstimatorEKF::findInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped& initial_guess)
{
    logger_->info("Finding initial pose using observation data...");
    double applied_gyro_bias = params_.initial_gyro_bias_z;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (is_init_.load()) {
            applied_gyro_bias = current_state_.mu(5);
        }
        current_state_.setPose(
            initial_guess.pose.pose.position.x,
            initial_guess.pose.pose.position.y,
            {
                initial_guess.pose.pose.orientation.w,
                initial_guess.pose.pose.orientation.x,
                initial_guess.pose.pose.orientation.y,
                initial_guess.pose.pose.orientation.z
            },
            0.0,    // initial velocity
            0.0,    // initial angular velocity
            applied_gyro_bias
        );
        // Initialize covariance matrix with appropriate initial uncertainty
        // Reset covariance to reflect initial pose uncertainty for full state (robot + landmark)
        int total_state_size = State::getTotalStateSize();  // robot(6) + landmark(3)
        int robot_state_size = State::getRobotStateSize();
        current_state_.sigma = Eigen::MatrixXd::Zero(total_state_size, total_state_size);

        auto pose_variance_or_default = [](double variance, double default_variance) {
            return variance > 1.0e-9 ? variance : default_variance;
        };

        // Use pose command covariance when provided. Floor-transition poses are
        // expected to be accurate, but still non-zero to avoid infinite trust.
        current_state_.sigma(0, 0) = pose_variance_or_default(initial_guess.pose.covariance[0], 0.01);
        current_state_.sigma(1, 1) = pose_variance_or_default(initial_guess.pose.covariance[7], 0.01);
        current_state_.sigma(2, 2) = pose_variance_or_default(initial_guess.pose.covariance[35], 0.0076);
        current_state_.sigma(3, 3) = 0.002 * 0.002;  // v variance (2mm/s std)
        current_state_.sigma(4, 4) = (0.05 * M_PI / 180.0) * (0.05 * M_PI / 180.0);  // w variance (0.05°/s std)
        current_state_.sigma(5, 5) = (0.5 * M_PI / 180.0) * (0.5 * M_PI / 180.0);  // gyro bias variance (0.5°/s std)

        // Set landmark initial covariance (large uncertainty, independent of robot)
        current_state_.sigma(robot_state_size + 0, robot_state_size + 0) = 100.0;         // lm_x variance (10m std)
        current_state_.sigma(robot_state_size + 1, robot_state_size + 1) = 100.0;         // lm_y variance (10m std)
        current_state_.sigma(robot_state_size + 2, robot_state_size + 2) = 1.57 * 1.57;   // lm_yaw variance (90° std)

        // Use node's current time, NOT message stamp.
        // initial_guess.header.stamp (e.g., from RViz) can be far from sensor timestamps,
        // causing enormous dt on the first IMU/wheel callback → Kalman gain blow-up.
        current_state_.time = node_->now();
    }

    {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        has_new_wheel_ = false;
        has_new_imu_ = false;
        latest_wheel_odom_.reset();
        latest_imu_.reset();
        wheel_odom_queue_.clear();
        imu_queue_.clear();
    }
    lm_covariance_history_.clear();
    landmark_initialized_ = false;

    auto angle = Eigen::Quaterniond(initial_guess.pose.pose.orientation.w,
                        initial_guess.pose.pose.orientation.x,
                        initial_guess.pose.pose.orientation.y,
                        initial_guess.pose.pose.orientation.z);
    auto time = rclcpp::Time(initial_guess.header.stamp);

    // The initial pose used by this odom_estimator is an odom-frame seed for the
    // EKF state. Keep map->odom as identity here to avoid applying the same
    // pose both in state (odom->base_link) and in TF (map->odom).
    Eigen::Vector3d map_to_odom_pos(0.0, 0.0, 0.0);
    Eigen::Quaterniond map_to_odom_rot(1.0, 0.0, 0.0, 0.0);
    updateMapToOdomTransform(map_to_odom_pos, map_to_odom_rot, time);

    // Mark as initialized after finding the pose
    is_init_.store(true);

    Eigen::Matrix3d R = angle.toRotationMatrix();
    double yaw = std::atan2(R(1, 0), R(0, 0));
    logger_->info("Initial pose set at x={:.3f}, y={:.3f}, yaw={:.3f}°, gyro_bias_z={:.6f}°/s",
                  initial_guess.pose.pose.position.x,
                  initial_guess.pose.pose.position.y,
                  yaw * 180.0 / M_PI,
                  applied_gyro_bias * 180.0 / M_PI);
}

void OdomEstimatorEKF::processInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
{
    logger_->info("Received initial pose, finding actual initial pose...");

    // Use findInitPose to process the initial guess with observation data
    findInitPose(*pose);
    auto odom = state2Msg();

    // Publish the refined pose and initial map->odom transform
    triggerNotify(INIT_POSE_PROCESSED_EVENT, std::make_any<nav_msgs::msg::Odometry>(odom));
    logger_->info("Initial pose found and set! OdomEstimator is now initialized.");
}

void OdomEstimatorEKF::processSensorFusionTick()
{
    if (!is_init_) {
        return;
    }

    if (is_doing_odom_estimation_.exchange(true)) {
        logger_->debug("[processSensorFusionTick] State estimation in progress, skipping tick");
        return;
    }

    std::deque<nav_msgs::msg::Odometry::SharedPtr> wheel_queue;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue;
    {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        wheel_queue.swap(wheel_odom_queue_);
        imu_queue.swap(imu_queue_);
        has_new_wheel_ = false;
        has_new_imu_ = false;
    }

    if (wheel_queue.empty() && imu_queue.empty()) {
        is_doing_odom_estimation_.store(false);
        return;
    }

    double last_v;
    double last_w;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_v = current_state_.mu(3);
        last_w = current_state_.mu(4);
    }

    auto stamp_of_wheel = [](const nav_msgs::msg::Odometry::SharedPtr& msg) {
        return rclcpp::Time(msg->header.stamp);
    };
    auto stamp_of_imu = [](const sensor_msgs::msg::Imu::SharedPtr& msg) {
        return rclcpp::Time(msg->header.stamp);
    };

    auto process_until = [&](const rclcpp::Time& measurement_time,
                             const geometry_msgs::msg::Twist::SharedPtr& cmd_vel) {
        rclcpp::Time state_time;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_time = current_state_.time;
        }

        const double dt = (measurement_time - state_time).seconds();
        if (dt < -1e-4) {
            logger_->debug("[processSensorFusionTick] Skipping stale measurement: dt={:.6f}", dt);
            return false;
        }

        if (!prediction(cmd_vel, dt)) {
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_.time = measurement_time;
        }
        return true;
    };

    while (!wheel_queue.empty() || !imu_queue.empty()) {
        const bool use_wheel = !wheel_queue.empty() &&
                               (imu_queue.empty() || stamp_of_wheel(wheel_queue.front()) <= stamp_of_imu(imu_queue.front()));

        auto cmd_vel = std::make_shared<geometry_msgs::msg::Twist>();
        cmd_vel->linear.x = last_v;
        cmd_vel->angular.z = last_w;

        if (use_wheel) {
            auto wheel_odom = wheel_queue.front();
            wheel_queue.pop_front();
            last_v = wheel_odom->twist.twist.linear.x;
            cmd_vel->linear.x = last_v;

            if (process_until(stamp_of_wheel(wheel_odom), cmd_vel) && !correctionWheel(wheel_odom)) {
                logger_->warn("[processSensorFusionTick] Wheel correction failed");
            }
        } else {
            auto imu = imu_queue.front();
            imu_queue.pop_front();
            last_w = imu->angular_velocity.z;
            cmd_vel->angular.z = last_w;

            if (process_until(stamp_of_imu(imu), cmd_vel) && !correctionImu(imu)) {
                logger_->warn("[processSensorFusionTick] IMU correction failed");
            }
        }
    }

    auto result_odom = state2Msg();
    triggerNotify(WHEEL_PROCESSED_EVENT, std::make_any<nav_msgs::msg::Odometry>(result_odom));

    is_doing_odom_estimation_.store(false);
}

void OdomEstimatorEKF::processLandmarkObservation(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr observation)
{
    if (!is_landmark_correction_enabled_.load()) {
        return;
    }

    if (!is_init_) {
        logger_->debug("[processLandmarkObservation] OdomEstimator not initialized, skipping observation");
        return;
    }

    if (is_doing_odom_estimation_.exchange(true)) {
        logger_->debug("[processLandmarkObservation] State estimation in progress, skipping observation");
        return;
    }

    if (!landmark_initialized_) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        Eigen::Quaterniond q_meas(
            observation->pose.pose.orientation.w,
            observation->pose.pose.orientation.x,
            observation->pose.pose.orientation.y,
            observation->pose.pose.orientation.z);
        Eigen::Matrix3d rotation = q_meas.toRotationMatrix();
        const double measured_yaw_baselink = std::atan2(rotation(1, 0), rotation(0, 0));
        const double measured_x_baselink = observation->pose.pose.position.x;
        const double measured_y_baselink = observation->pose.pose.position.y;

        const double robot_x = current_state_.mu(0);
        const double robot_y = current_state_.mu(1);
        const double robot_yaw = current_state_.mu(2);
        const double cos_theta = std::cos(robot_yaw);
        const double sin_theta = std::sin(robot_yaw);

        double lm_x_odom = robot_x + cos_theta * measured_x_baselink - sin_theta * measured_y_baselink;
        double lm_y_odom = robot_y + sin_theta * measured_x_baselink + cos_theta * measured_y_baselink;
        double lm_yaw_odom = robot_yaw + measured_yaw_baselink;
        lm_yaw_odom = std::atan2(std::sin(lm_yaw_odom), std::cos(lm_yaw_odom));

        const int robot_state_size = State::getRobotStateSize();
        current_state_.mu(robot_state_size + 0) = lm_x_odom;
        current_state_.mu(robot_state_size + 1) = lm_y_odom;
        current_state_.mu(robot_state_size + 2) = lm_yaw_odom;

        current_state_.sigma(robot_state_size + 0, robot_state_size + 0) = 0.03 * 0.03;
        current_state_.sigma(robot_state_size + 1, robot_state_size + 1) = 0.03 * 0.03;
        current_state_.sigma(robot_state_size + 2, robot_state_size + 2) = std::pow(2.0 * M_PI / 180.0, 2);
        landmark_initialized_ = true;

        logger_->info("[processLandmarkObservation] Landmark initialized at odom frame: x={:.3f}, y={:.3f}, yaw={:.3f}°",
                      lm_x_odom, lm_y_odom, lm_yaw_odom * 180.0 / M_PI);
    } else if (!correctionLandmarkObservation(observation)) {
        logger_->warn("[processLandmarkObservation] Landmark correction failed");
    }

    auto result_odom = state2Msg();
    triggerNotify(POINT_CLOUD_PROCESSED_EVENT, std::make_any<nav_msgs::msg::Odometry>(result_odom));

    is_doing_odom_estimation_.store(false);
}

void OdomEstimatorEKF::processWheel(const nav_msgs::msg::Odometry::SharedPtr odom) // with update
{
    if (!odom) {
        return;
    }

    if (!is_init_) {
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header = odom->header;
        initial_pose.pose = odom->pose;
        findInitPose(initial_pose);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_.time = rclcpp::Time(odom->header.stamp);
        }
    }

    {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        latest_wheel_odom_ = odom;
        wheel_odom_queue_.push_back(odom);
        if (wheel_odom_queue_.size() > 2000) {
            wheel_odom_queue_.pop_front();
        }
        has_new_wheel_ = true;
    }

    logger_->debug("[processWheel] Stored latest wheel odometry: v={:.3f} m/s", odom->twist.twist.linear.x);
}

void OdomEstimatorEKF::processImu(const sensor_msgs::msg::Imu::SharedPtr imu) // with update
{
    if (!imu) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        latest_imu_ = imu;
        imu_queue_.push_back(imu);
        if (imu_queue_.size() > 10000) {
            imu_queue_.pop_front();
        }
        has_new_imu_ = true;
    }

    logger_->debug("[processImu] Stored latest IMU gyro: w={:.3f}°/s", imu->angular_velocity.z * 180.0 / M_PI);
}

bool OdomEstimatorEKF::prediction(const geometry_msgs::msg::Twist::SharedPtr cmd_vel, double dt)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (dt < -1e-4) {
        logger_->warn("[prediction] Prediction rejected: negative dt = {:.4f}", dt);
        return false;
    }

    if (dt < 1e-4) {
        logger_->debug("[prediction] Prediction skipped for small dt = {:.4f}; correction may still run", dt);
        return true;
    }

    const double ANGLE_THRESHOLD = 1e-4;

    // Extract velocities from Twist message
    double v = cmd_vel->linear.x;
    double w = cmd_vel->angular.z;

    double gyro_bias = current_state_.mu(5);
    double corrected_w = w - gyro_bias;
    double theta = current_state_.mu(2);
    double dx, dy, dtheta;

    // Determine if this is a spin turn (in-place rotation) or normal Ackermann drive
    // Spin turn: very low linear velocity but significant angular velocity
    if (std::fabs(v) < 1e-3 && std::fabs(corrected_w) > ANGLE_THRESHOLD) {
        // === Spin Turn Mode ===
        // Robot rotates in place around its center
        // Use angular velocity directly
        dtheta = corrected_w * dt;

        // For spin turn, position change is minimal (rotating in place)
        dx = 0.0;
        dy = 0.0;

        logger_->debug("[prediction] Spin turn mode: raw_w={:.3f}°/s bias={:.3f}°/s corrected_w={:.3f}°/s dθ={:.4f}° dt={:.4f}",
                       w * 180.0 / M_PI, gyro_bias * 180.0 / M_PI, corrected_w * 180.0 / M_PI, dtheta * 180.0 / M_PI, dt);

        // Jacobians for spin turn
        int state_size = current_state_.mu.size();
        Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(state_size, state_size);
        // For pure rotation, position doesn't change with theta
        Gt(0, 2) = 0.0;
        Gt(1, 2) = 0.0;
        // Position doesn't depend on velocities in spin turn
        Gt(0, 3) = 0.0;  // ∂x/∂v = 0 (no translation)
        Gt(1, 3) = 0.0;  // ∂y/∂v = 0 (no translation)
        Gt(0, 4) = 0.0;  // ∂x/∂w = 0 (no translation)
        Gt(1, 4) = 0.0;  // ∂y/∂w = 0 (no translation)
        // Orientation depends on angular velocity and gyro bias.
        Gt(2, 4) = dt;   // ∂theta/∂w = dt (dtheta = w * dt)
        Gt(2, 5) = -dt;  // ∂theta/∂bias = -dt
        // Velocities remain as identity (v_new = v_old, w_new = w_old)

        // Process noise for spin turn
        // Q_process is velocity-dependent to model higher uncertainty at higher speeds
        Eigen::MatrixXd Q_process = Eigen::MatrixXd::Zero(state_size, state_size);
        Q_process(0, 0) = 0.001;  // x position uncertainty (minimal for spin turn)
        Q_process(1, 1) = 0.001;  // y position uncertainty (minimal for spin turn)
        Q_process(2, 2) = std::max(0.02 * std::fabs(corrected_w) * dt, 0.001);  // θ uncertainty proportional to angular velocity
        Q_process(3, 3) = std::max(0.05 * std::fabs(v), 0.01*0.01);   // v uncertainty (minimal in spin turn)
        Q_process(4, 4) = std::max(0.05 * std::fabs(w), 1.0 * M_PI/180.0 * 1.0 * M_PI/180.0);    // w uncertainty proportional to angular velocity
        Q_process(5, 5) = std::pow(0.005 * M_PI / 180.0, 2);  // gyro bias random walk
        // Landmarks don't have process noise

        // 공분산 예측
        current_state_.sigma = Gt * current_state_.sigma * Gt.transpose() + Q_process;
    } else {
        // === Normal Ackermann Drive Mode ===
        // motion model

        if (std::fabs(corrected_w) > ANGLE_THRESHOLD) {
            // 회전 주행 (curved motion)
            double R = v / corrected_w;  // Direct calculation, safer than using tan(delta)
            dtheta = corrected_w * dt;
            dx = R * (sin(theta + dtheta) - sin(theta));
            dy = -R * (cos(theta + dtheta) - cos(theta));
        } else {
            // 직선 주행 (straight line motion)
            dtheta = 0.0;
            dx = v * cos(theta) * dt;
            dy = v * sin(theta) * dt;
        }

        logger_->debug("[prediction] Ackermann mode: v={:.3f} raw_w={:.3f} bias={:.3f} corrected_w={:.3f} dθ={:.4f} dt={:.4f}",
                       v, w, gyro_bias, corrected_w, dtheta, dt);

        // Jacobians for Ackermann steering - must match state size for SLAM mode
        int state_size = current_state_.mu.size();
        Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(state_size, state_size);

        // Position derivatives w.r.t. theta
        Gt(0, 2) = -v * dt * sin(theta);
        Gt(1, 2) =  v * dt * cos(theta);

        // Position derivatives w.r.t. v (linear velocity)
        Gt(0, 3) = dt * cos(theta);
        Gt(1, 3) = dt * sin(theta);

        // Position and orientation derivatives w.r.t. w (angular velocity)
        if (std::fabs(corrected_w) > ANGLE_THRESHOLD) {
            // Curved motion
            double R = v / corrected_w;
            Gt(0, 4) = dt * R * cos(theta + corrected_w * dt) - (v / (corrected_w * corrected_w)) * (sin(theta + corrected_w * dt) - sin(theta));
            Gt(1, 4) = dt * R * sin(theta + corrected_w * dt) + (v / (corrected_w * corrected_w)) * (cos(theta + corrected_w * dt) - cos(theta));
            Gt(2, 4) = dt;
            Gt(0, 5) = -Gt(0, 4);
            Gt(1, 5) = -Gt(1, 4);
            Gt(2, 5) = -dt;
        } else {
            // Straight motion
            Gt(0, 4) = 0.0;
            Gt(1, 4) = 0.0;
            Gt(2, 4) = dt;
            Gt(0, 5) = 0.0;
            Gt(1, 5) = 0.0;
            Gt(2, 5) = -dt;
        }

        // Process noise for Ackermann motion
        // Q_process is velocity-dependent to model higher uncertainty at higher speeds
        Eigen::MatrixXd Q_process = Eigen::MatrixXd::Zero(state_size, state_size);
        Q_process(0, 0) = std::max(0.01 * std::fabs(v) * dt, 0.001);  // x uncertainty proportional to velocity
        Q_process(1, 1) = std::max(0.01 * std::fabs(v) * dt, 0.001);  // y uncertainty proportional to velocity
        Q_process(2, 2) = std::max(0.02 * std::fabs(corrected_w) * dt, 0.001);  // θ uncertainty proportional to angular velocity
        Q_process(3, 3) = std::max(0.01 * std::fabs(v), 0.001);       // v uncertainty proportional to velocity
        Q_process(4, 4) = std::max(0.05 * std::fabs(w), 0.01);        // w uncertainty proportional to angular velocity
        Q_process(5, 5) = std::pow(0.005 * M_PI / 180.0, 2);          // gyro bias random walk
        // Landmarks don't have process noise

        // 공분산 예측
        current_state_.sigma = Gt * current_state_.sigma * Gt.transpose() + Q_process;
    }

    // 상태 예측 (공통)
    current_state_.mu(0) += dx;
    current_state_.mu(1) += dy;
    current_state_.mu(2) += dtheta;
    current_state_.mu(3) = v;   // Update velocity
    current_state_.mu(4) = w;   // Raw angular velocity state; prediction uses w - gyro_bias.

    logger_->debug("[prediction] Predicted state: x={:.3f}, y={:.3f}, yaw={:.3f}°, v={:.3f}, raw_w={:.3f}°/s, bias={:.3f}°/s",
                  current_state_.mu(0),
                  current_state_.mu(1),
                  current_state_.mu(2) * 180.0 / M_PI,
                  current_state_.mu(3),
                  current_state_.mu(4) * 180.0 / M_PI,
                  current_state_.mu(5) * 180.0 / M_PI);

    // Normalize yaw
    current_state_.mu(2) = atan2(sin(current_state_.mu(2)), cos(current_state_.mu(2)));

    return true;
}

bool OdomEstimatorEKF::correctionLandmarkObservation(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr observation)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!is_landmark_correction_enabled_.load()) {
        logger_->warn("[correctionLandmarkObservation] Landmark correction is disabled");
        return false;
    }

    int state_size = current_state_.mu.size();
    int robot_state_size = State::getRobotStateSize();  // x, y, yaw, v, w, gyro_bias

    // Extract landmark measurement from pose message (in base_link frame)
    Eigen::Quaterniond q_meas(
        observation->pose.pose.orientation.w,
        observation->pose.pose.orientation.x,
        observation->pose.pose.orientation.y,
        observation->pose.pose.orientation.z
    );
    Eigen::Matrix3d rotation_matrix = q_meas.toRotationMatrix();
    double measured_yaw_baselink = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    double measured_x_baselink = observation->pose.pose.position.x;
    double measured_y_baselink = observation->pose.pose.position.y;

    // Measurement vector z = [lm_x, lm_y, lm_yaw] in base_link frame
    Eigen::VectorXd z(3);
    z(0) = measured_x_baselink;
    z(1) = measured_y_baselink;
    z(2) = std::atan2(std::sin(measured_yaw_baselink), std::cos(measured_yaw_baselink));  // Normalize measured yaw

    // Get current robot pose from state (in odom frame)
    double robot_x = current_state_.mu(0);
    double robot_y = current_state_.mu(1);
    double robot_yaw = current_state_.mu(2);

    // Get landmark pose from state (in odom frame)
    double lm_x_odom = current_state_.mu(robot_state_size + 0);
    double lm_y_odom = current_state_.mu(robot_state_size + 1);
    double lm_yaw_odom = current_state_.mu(robot_state_size + 2);

    logger_->debug("[correctionLandmarkObservation] Robot pose (odom): x={:.3f}, y={:.3f}, yaw={:.3f}°",
                robot_x, robot_y, robot_yaw * 180.0 / M_PI);
    logger_->debug("[correctionLandmarkObservation] Landmark state (odom): x={:.3f}, y={:.3f}, yaw={:.3f}°",
                lm_x_odom, lm_y_odom, lm_yaw_odom * 180.0 / M_PI);
    logger_->debug("[correctionLandmarkObservation] Landmark measured (base_link): x={:.3f}, y={:.3f}, yaw={:.3f}°",
                z(0), z(1), z(2) * 180.0 / M_PI);

    // Predicted measurement: h(μ̄) = transform landmark from odom to base_link
    // This is the inverse of what we did in initialization
    double cos_theta = std::cos(robot_yaw);
    double sin_theta = std::sin(robot_yaw);

    // Transform landmark from odom to base_link: h = R(-θ) * (lm_odom - robot_odom)
    double dx_odom = lm_x_odom - robot_x;
    double dy_odom = lm_y_odom - robot_y;

    Eigen::VectorXd h_mu(3);
    h_mu(0) =  cos_theta * dx_odom + sin_theta * dy_odom;  // x in base_link
    h_mu(1) = -sin_theta * dx_odom + cos_theta * dy_odom;  // y in base_link
    h_mu(2) = lm_yaw_odom - robot_yaw;                      // yaw relative to robot

    // Normalize predicted yaw to [-π, π]
    h_mu(2) = std::atan2(std::sin(h_mu(2)), std::cos(h_mu(2)));

    logger_->debug("[correctionLandmarkObservation] Predicted measurement h(μ̄) (base_link): x={:.3f}, y={:.3f}, yaw={:.3f}°",
                h_mu(0), h_mu(1), h_mu(2) * 180.0 / M_PI);

    // Measurement Jacobian H for landmark observation in base_link frame
    // h(x) = R(-θ) * (lm_odom - robot_odom) + lm_yaw - robot_yaw
    // This is the standard EKF-SLAM measurement model
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state_size);

    // ∂h/∂robot_x: h_x depends on robot position through transformation
    H(0, 0) = -cos_theta;  // ∂h_x/∂robot_x
    H(1, 0) =  sin_theta;  // ∂h_y/∂robot_x

    // ∂h/∂robot_y: h_y depends on robot position through transformation
    H(0, 1) = -sin_theta;  // ∂h_x/∂robot_y
    H(1, 1) = -cos_theta;  // ∂h_y/∂robot_y

    // ∂h/∂robot_yaw: rotation transformation depends on robot yaw
    H(0, 2) = -sin_theta * dx_odom + cos_theta * dy_odom;  // ∂h_x/∂robot_yaw
    H(1, 2) = -cos_theta * dx_odom - sin_theta * dy_odom;  // ∂h_y/∂robot_yaw
    H(2, 2) = -1.0;  // ∂h_yaw/∂robot_yaw

    // ∂h/∂lm_x, ∂h/∂lm_y, ∂h/∂lm_yaw: h depends on landmark state
    H(0, robot_state_size + 0) =  cos_theta;  // ∂h_x/∂lm_x
    H(1, robot_state_size + 0) = -sin_theta;  // ∂h_y/∂lm_x

    H(0, robot_state_size + 1) =  sin_theta;  // ∂h_x/∂lm_y
    H(1, robot_state_size + 1) =  cos_theta;  // ∂h_y/∂lm_y

    H(2, robot_state_size + 2) = 1.0;  // ∂h_yaw/∂lm_yaw

    // Measurement noise covariance R (3x3 matrix)
    // Extract from V-marker pose covariance matrix
    // PoseWithCovariance.covariance is 6x6 in row-major order: [x, y, z, roll, pitch, yaw]
    // We need x (index 0), y (index 7), yaw (index 35)
    Eigen::MatrixXd R(3, 3);
    R.setZero();
    R(0, 0) = observation->pose.covariance[0];   // x variance (0,0)
    R(1, 1) = observation->pose.covariance[7];   // y variance (1,1)
    R(2, 2) = observation->pose.covariance[35];  // yaw variance (5,5)

    // Ensure minimum variance to avoid numerical issues
    if (R(0, 0) < 1e-6) R(0, 0) = 0.001;  // standard deviation ~3 cm
    if (R(1, 1) < 1e-6) R(1, 1) = 0.001;  // standard deviation ~3 cm
    if (R(2, 2) < 1e-6) R(2, 2) = 0.0005; // radian variance, deg: ~1.3°

    logger_->debug("[correctionLandmarkObservation] Landmark measurement covariance R: x={:.6f}, y={:.6f}, yaw={:.6f}",
                  R(0, 0), R(1, 1), R(2, 2));

    // Kalman gain: K_t = Σ̄_t * H^T * (H * Σ̄_t * H^T + R)^{-1}
    // Innovation covariance S includes:
    // - H * Σ̄ * H^T: Uncertainty from predicted robot & landmark states
    // - R: landmark measurement noise
    Eigen::MatrixXd S = H * current_state_.sigma * H.transpose() + R;

    // Log innovation covariance to show total uncertainty
    logger_->debug("[correctionLandmarkObservation] Innovation covariance S (includes prediction uncertainty): x={:.6f}, y={:.6f}, yaw={:.6f}",
                  S(0, 0), S(1, 1), S(2, 2));
    logger_->debug("[correctionLandmarkObservation] Prediction uncertainty contribution: x={:.6f}, y={:.6f}, yaw={:.6f}",
                  S(0, 0) - R(0, 0), S(1, 1) - R(1, 1), S(2, 2) - R(2, 2));

    Eigen::MatrixXd K = current_state_.sigma * H.transpose() * S.inverse();

    bool is_tracking_mode = landmark_initialized_;

    if (is_tracking_mode) {
        // TRACKING MODE: Update only robot state, keep landmark fixed
        // Zero out landmark columns in Jacobian H (∂h/∂landmark = 0)
        // This means landmark state doesn't affect predicted measurement
        // Only robot state affects h(x), so only robot state gets updated
        H.block(0, robot_state_size, 3, 3).setZero();  // Zero columns for landmark states
        logger_->debug("[correctionLandmarkObservation] Tracking mode: landmark frozen, only robot state will be updated");

        // Recompute S and K with modified H
        S = H * current_state_.sigma * H.transpose() + R;
        K = current_state_.sigma * H.transpose() * S.inverse();
    }

    // Innovation (measurement residual): z_t - h(μ̄_t)
    Eigen::VectorXd innovation = z - h_mu;

    // Normalize angle difference in innovation
    innovation(2) = std::atan2(std::sin(innovation(2)), std::cos(innovation(2)));

    logger_->info("[correctionLandmarkObservation] Landmark innovation: x={:.3f}, y={:.3f}, yaw={:.3f}° (mode: {})",
                 innovation(0), innovation(1), innovation(2) * 180.0 / M_PI,
                 is_tracking_mode ? "tracking" : "initialization");

    // Update state: μ_t = μ̄_t + K_t * (z_t - h(μ̄_t))
    // K is state_size × 3, innovation is 3 × 1, so correction is state_size × 1

    logger_->info("[correctionLandmarkObservation] Robot state before correction: x={:.3f}, y={:.3f}, yaw={:.3f}°",
                current_state_.mu(0), current_state_.mu(1), current_state_.mu(2) * 180.0 / M_PI);
    logger_->debug("[correctionLandmarkObservation] Landmark state (should be frozen in tracking): x={:.3f}, y={:.3f}, yaw={:.3f}°",
                current_state_.mu(robot_state_size + 0), current_state_.mu(robot_state_size + 1),
                current_state_.mu(robot_state_size + 2) * 180.0 / M_PI);

    Eigen::VectorXd correction = K * innovation;
    current_state_.mu += correction;  // Update robot state (and landmarks in init mode)

    // Normalize robot yaw after update
    current_state_.mu(2) = std::atan2(std::sin(current_state_.mu(2)), std::cos(current_state_.mu(2)));

    // Normalize landmark yaw after update
    current_state_.mu(robot_state_size + 2) = std::atan2(
        std::sin(current_state_.mu(robot_state_size + 2)),
        std::cos(current_state_.mu(robot_state_size + 2))
    );

    logger_->info("[correctionLandmarkObservation] Robot state after correction: x={:.3f}, y={:.3f}, yaw={:.3f}°",
                 current_state_.mu(0), current_state_.mu(1), current_state_.mu(2) * 180.0 / M_PI);
    logger_->debug("[correctionLandmarkObservation] Landmark state after (verify frozen): x={:.3f}, y={:.3f}, yaw={:.3f}°",
                current_state_.mu(robot_state_size + 0), current_state_.mu(robot_state_size + 1),
                current_state_.mu(robot_state_size + 2) * 180.0 / M_PI);

    // Update covariance: Σ_t = (I - K_t * H_t) * Σ̄_t
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_size, state_size);
    current_state_.sigma = (I - K * H) * current_state_.sigma;

    // Check if landmark covariance has stabilized (converged) - only in initialization mode
    if (!is_tracking_mode) {
        // INITIALIZATION MODE: Check for landmark convergence
        // Extract landmark covariance (3x3 submatrix from full state covariance)
        double lm_x_var = current_state_.sigma(robot_state_size + 0, robot_state_size + 0);
        double lm_y_var = current_state_.sigma(robot_state_size + 1, robot_state_size + 1);
        double lm_yaw_var = current_state_.sigma(robot_state_size + 2, robot_state_size + 2);

        // Compute covariance norm (Frobenius norm of diagonal elements)
        double covariance_norm = std::sqrt(lm_x_var + lm_y_var + lm_yaw_var);

        // Add to history
        lm_covariance_history_.push_back(covariance_norm);
        if (lm_covariance_history_.size() > CONVERGENCE_WINDOW_SIZE) {
            lm_covariance_history_.erase(lm_covariance_history_.begin());
        }

        // Check for stability: if we have enough samples and variance is stable
        if (lm_covariance_history_.size() >= CONVERGENCE_WINDOW_SIZE) {
            double mean = 0.0;
            for (double val : lm_covariance_history_) {
                mean += val;
            }
            mean /= lm_covariance_history_.size();

            double max_deviation = 0.0;
            for (double val : lm_covariance_history_) {
                max_deviation = std::max(max_deviation, std::abs(val - mean) / mean);
            }

            // Convergence criteria:
            // 1. Relative stability: covariance deviation < threshold (e.g., 5%)
            // 2. Absolute uncertainty: position std < 0.1m AND orientation std < 5°
            bool is_relatively_stable = (max_deviation < CONVERGENCE_THRESHOLD);
            bool is_absolutely_accurate = (std::sqrt(lm_x_var) < 0.1 &&
                                           std::sqrt(lm_y_var) < 0.1 &&
                                           std::sqrt(lm_yaw_var) < 5.0 * M_PI / 180.0);

            if (is_relatively_stable && is_absolutely_accurate) {
                logger_->info("[correctionLandmarkObservation] Landmark converged: x_std={:.4f}m, y_std={:.4f}m, yaw_std={:.2f}° (deviation={:.2f}%) - Switching to tracking mode",
                             std::sqrt(lm_x_var), std::sqrt(lm_y_var), std::sqrt(lm_yaw_var) * 180.0 / M_PI, max_deviation * 100.0);
                landmark_initialized_ = true;
                lm_covariance_history_.clear();  // Reset for potential re-initialization
            } else {
                logger_->debug("[correctionLandmarkObservation] Landmark covariance: x_std={:.4f}m, y_std={:.4f}m, yaw_std={:.2f}° (stable={}, accurate={}, deviation={:.2f}%)",
                             std::sqrt(lm_x_var), std::sqrt(lm_y_var), std::sqrt(lm_yaw_var) * 180.0 / M_PI,
                             is_relatively_stable ? "Y" : "N",
                             is_absolutely_accurate ? "Y" : "N",
                             max_deviation * 100.0);
            }
        } else {
            logger_->debug("[correctionLandmarkObservation] Landmark covariance: x_std={:.4f}m, y_std={:.4f}m, yaw_std={:.2f}° (collecting data: {}/{})",
                         std::sqrt(lm_x_var), std::sqrt(lm_y_var), std::sqrt(lm_yaw_var) * 180.0 / M_PI,
                         lm_covariance_history_.size(), CONVERGENCE_WINDOW_SIZE);
        }
    } else {
        // TRACKING MODE: Log landmark state for monitoring
        double lm_x_var = current_state_.sigma(robot_state_size + 0, robot_state_size + 0);
        double lm_y_var = current_state_.sigma(robot_state_size + 1, robot_state_size + 1);
        double lm_yaw_var = current_state_.sigma(robot_state_size + 2, robot_state_size + 2);

        logger_->debug("[correctionLandmarkObservation] Tracking mode - Landmark frozen at: x={:.3f}m (std={:.4f}), y={:.3f}m (std={:.4f}), yaw={:.3f}° (std={:.2f}°)",
                     current_state_.mu(robot_state_size + 0), std::sqrt(lm_x_var),
                     current_state_.mu(robot_state_size + 1), std::sqrt(lm_y_var),
                     current_state_.mu(robot_state_size + 2) * 180.0 / M_PI, std::sqrt(lm_yaw_var) * 180.0 / M_PI);
    }

    logger_->debug("[correctionLandmarkObservation] Correction completed successfully (mode: {})",
                  is_tracking_mode ? "tracking" : "initialization");

    return true;
}


bool OdomEstimatorEKF::correctionImu(const sensor_msgs::msg::Imu::SharedPtr imu)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Extract angular velocity measurement from IMU
    double measured_w = imu->angular_velocity.z;

    // Keep mu(4) as the latest raw gyro measurement. The bias state is not
    // independently observable while the robot is moving, so do not run a
    // w+bias Kalman correction here. Prediction uses (raw_w - bias).
    current_state_.mu(4) = measured_w;
    current_state_.sigma(4, 4) = std::pow(0.5 * M_PI / 180.0, 2);

    // Stationary bias update: if wheel velocity says the robot is stopped and
    // gyro rate is small, treat the measured gyro rate as bias and adapt slowly.
    const double stationary_v_threshold = 0.005;                 // m/s
    const double stationary_w_threshold = 0.2 * M_PI / 180.0;    // rad/s
    if (std::fabs(current_state_.mu(3)) < stationary_v_threshold &&
        std::fabs(measured_w) < stationary_w_threshold)
    {
        const double alpha = 0.001;
        const double previous_bias = current_state_.mu(5);
        current_state_.mu(5) = (1.0 - alpha) * current_state_.mu(5) + alpha * measured_w;
        current_state_.sigma(5, 5) = std::min(
            current_state_.sigma(5, 5),
            std::pow(0.2 * M_PI / 180.0, 2));

        logger_->debug("[correctionImu] Stationary gyro bias update: measured={:.4f}°/s, bias={:.4f}°/s, dbias={:.4f}°/s",
                      measured_w * 180.0 / M_PI,
                      current_state_.mu(5) * 180.0 / M_PI,
                      (current_state_.mu(5) - previous_bias) * 180.0 / M_PI);
    }

    logger_->debug("[correctionImu] IMU gyro stored: v={:.3f}m/s, raw_w={:.3f}°/s, bias={:.3f}°/s, corrected_w={:.3f}°/s (yaw unchanged: {:.3f}°)",
                  current_state_.mu(3),
                  current_state_.mu(4) * 180.0 / M_PI,
                  current_state_.mu(5) * 180.0 / M_PI,
                  (current_state_.mu(4) - current_state_.mu(5)) * 180.0 / M_PI,
                  current_state_.mu(2) * 180.0 / M_PI);

    return true;
}

bool OdomEstimatorEKF::correctionWheel(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Extract linear velocity measurement from odometry
    double measured_v = odom->twist.twist.linear.x;

    // Measurement vector z = [v] - only measuring linear velocity
    Eigen::VectorXd z(1);
    z(0) = measured_v;

    // Predicted measurement: h(μ̄) = [μ̄_v]
    Eigen::VectorXd h_mu(1);
    h_mu(0) = current_state_.mu(3);  // predicted v

    // Measurement Jacobian H for v-only observation
    // H is 1 × 5 (state_size), only element (0, 3) is non-zero
    int state_size = current_state_.mu.size();
    Eigen::MatrixXd H(1, state_size);
    H.setZero();
    H(0, 3) = 1.0;  // Observing v (fourth state element, index 3)

    // Measurement covariance: only v variance
    // Use appropriate variance for wheel odometry velocity measurement
    Eigen::MatrixXd R_subset(1, 1);
    R_subset(0, 0) = 0.04 * 0.04;  // wheel v variance from bag moving residuals

    // Ensure minimum variance for numerical stability
    if (R_subset(0, 0) < 1e-9) {
        logger_->warn("[correctionWheel] v measurement covariance too small ({:.6e}), using minimum: 1e-9", R_subset(0, 0));
        R_subset(0, 0) = 1e-9;
    }

    logger_->debug("[correctionWheel] Measured v: {:.3f} m/s, Predicted v: {:.3f} m/s, Variance: {:.6e} (std: {:.3f} m/s)",
                  measured_v, h_mu(0), R_subset(0, 0), std::sqrt(R_subset(0, 0)));

    // (1) Compute Kalman gain: K = (PH') / (HPH' + R)
    // Innovation covariance S = H * Σ̄ * H^T + R
    Eigen::MatrixXd pht = current_state_.sigma * H.transpose();  // state_size x 1 matrix
    Eigen::MatrixXd hphr = H * pht + R_subset;  // 1x1 scalar (innovation covariance)
    Eigen::MatrixXd hphr_inverse = hphr.inverse();  // 1x1
    Eigen::MatrixXd K = pht * hphr_inverse;  // state_size x 1 Kalman gain
    // Wheel odometry here is used as a forward-speed measurement only. Letting
    // covariance coupling move pose/yaw from a scalar v residual can distort the
    // trajectory, especially after adding gyro bias as a state.
    K(0) = 0.0;
    K(1) = 0.0;
    K(2) = 0.0;
    K(4) = 0.0;
    K(5) = 0.0;

    // Innovation (measurement residual): z_t - h(μ̄_t)
    Eigen::VectorXd innovation = z - h_mu;  // 1x1

    logger_->debug("[correctionWheel] Innovation (v): {:.3f} m/s", innovation(0));
    logger_->debug("[correctionWheel] Kalman Gain: x={:.4f}, y={:.4f}, yaw={:.4f}, v={:.4f}, w={:.4f}, bias={:.4f}",
                  K(0), K(1), K(2), K(3), K(4), K(5));

    // (2) Update state: μ_t = μ̄_t + K_t * (z_t - h(μ̄_t))
    // K is state_size x 1, innovation is 1x1, so correction is state_size x 1.
    Eigen::VectorXd correction = K * innovation;
    current_state_.mu += correction;  // Update ALL states (x, y, theta, v, w) through covariance correlation

    // Normalize yaw after update
    current_state_.mu(2) = std::atan2(std::sin(current_state_.mu(2)), std::cos(current_state_.mu(2)));

    logger_->debug("[correctionWheel] State correction: dx={:.4f}m, dy={:.4f}m, dtheta={:.4f}°, dv={:.4f}m/s, dw={:.4f}°/s",
                  correction(0), correction(1), correction(2) * 180.0 / M_PI, correction(3), correction(4) * 180.0 / M_PI);

    // (3) Update covariance using Joseph form: Σ_t = (I - K*H)*Σ̄*(I - K*H)^T + K*R*K^T
    // This form is more numerically stable than the simple form
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_size, state_size);
    Eigen::MatrixXd I_KH = I - K * H;
    current_state_.sigma = I_KH * current_state_.sigma * I_KH.transpose() + K * R_subset * K.transpose();

    logger_->debug("[correctionWheel] Odometry correction applied: v innovation={:.3f}m/s, corrected state=[x={:.3f}m, y={:.3f}m, theta={:.3f}°, v={:.3f}m/s, w={:.3f}°/s]",
                  innovation(0),
                  current_state_.mu(0), current_state_.mu(1), current_state_.mu(2) * 180.0 / M_PI,
                  current_state_.mu(3), current_state_.mu(4) * 180.0 / M_PI);

    return true;
}

void OdomEstimatorEKF::enableLandmarkCorrection()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (is_landmark_correction_enabled_.load()) {
        logger_->warn("Landmark correction is already enabled");
        return;
    }

    if (!is_init_.load()) {
        logger_->warn("Cannot enable landmark correction: odom_estimator not initialized");
        return;
    }

    // Landmark is already initialized in State constructor at (0, 0, 0) with large uncertainty
    // No need to add or initialize landmark here - it's always part of the state

    is_landmark_correction_enabled_.store(true);
    logger_->info("Landmark correction enabled - Landmark state already present (state size: {})",
                  current_state_.mu.size());
}

void OdomEstimatorEKF::disableLandmarkCorrection()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!is_landmark_correction_enabled_.load()) {
        logger_->warn("Already in localization-only mode");
        return;
    }

    if (!is_init_.load()) {
        logger_->warn("Cannot disable landmark correction: odom_estimator not initialized");
        return;
    }

    // Reset landmark state to (0, 0, 0) with large uncertainty
    // so that the next docking attempt starts fresh
    int robot_state_size = State::getRobotStateSize();
    current_state_.mu(robot_state_size + 0) = 0.0;
    current_state_.mu(robot_state_size + 1) = 0.0;
    current_state_.mu(robot_state_size + 2) = 0.0;

    current_state_.sigma(robot_state_size + 0, robot_state_size + 0) = 100.0;        // lm_x variance (10m std)
    current_state_.sigma(robot_state_size + 1, robot_state_size + 1) = 100.0;        // lm_y variance (10m std)
    current_state_.sigma(robot_state_size + 2, robot_state_size + 2) = 1.57 * 1.57;  // lm_yaw variance (90° std)

    // Zero cross-covariance between robot and landmark
    current_state_.sigma.block(0, robot_state_size, robot_state_size, 3).setZero();
    current_state_.sigma.block(robot_state_size, 0, 3, robot_state_size).setZero();

    // Clear convergence history and reset landmark initialization state
    lm_covariance_history_.clear();
    landmark_initialized_ = false;

    is_landmark_correction_enabled_.store(false);
    logger_->info("Landmark correction disabled - Landmark state reset to (0,0,0) with large uncertainty (state size: {})",
                  current_state_.mu.size());
}

bool OdomEstimatorEKF::isLandmarkInitialized() const
{
    // Check if in SLAM mode
    if (!is_landmark_correction_enabled_.load()) {
        return false;
    }

    return landmark_initialized_;
}


} // namespace odom_estimation
} // namespace gole
