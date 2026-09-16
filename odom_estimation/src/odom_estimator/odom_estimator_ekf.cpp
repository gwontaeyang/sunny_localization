#include "odom_estimation/odom_estimator/odom_estimator_ekf.hpp"
#include "odom_estimation/attitude_propagation.hpp"
#include "odom_estimation/measurement_validation.hpp"
#include "odom_estimation/motion_projection.hpp"

namespace gole {
namespace odom_estimation {

OdomEstimatorEKF::OdomEstimatorEKF(rclcpp_lifecycle::LifecycleNode* node,
                           std::shared_ptr<spdlog::logger> logger,
                           const Params& params)
    : OdomEstimatorBase(node, logger, params)
{
    sensor_fusion_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&OdomEstimatorEKF::processSensorFusionTick, this));
}

void OdomEstimatorEKF::findInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped& initial_guess)
{
    logger_->info("Finding initial pose using observation data...");
    std::array<double, 3> applied_gyro_bias{
        params_.initial_gyro_bias_x,
        params_.initial_gyro_bias_y,
        params_.initial_gyro_bias_z};
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (is_init_.load()) {
            applied_gyro_bias = {
                current_state_.mu(9),
                current_state_.mu(10),
                current_state_.mu(5)};
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
        if (params_.enable_3d_estimation) {
            current_state_.mu(6) = initial_guess.pose.pose.position.z;
        } else {
            current_state_.mu(6) = 0.0;
            current_state_.mu(7) = 0.0;
            current_state_.mu(8) = 0.0;
        }
        // Initialize covariance matrix with appropriate initial uncertainty
        // Reset covariance to reflect initial pose uncertainty for full state (robot + landmark)
        int total_state_size = State::getTotalStateSize();  // robot(11) + landmark(3)
        int robot_state_size = State::getRobotStateSize();
        current_state_.sigma = Eigen::MatrixXd::Zero(total_state_size, total_state_size);

        auto pose_variance_or_default = [](double variance, double default_variance) {
            return variance > 1.0e-9 ? variance : default_variance;
        };

        // Use pose command covariance when provided. Floor-transition poses are
        // expected to be accurate, but still non-zero to avoid infinite trust.
        current_state_.sigma(0, 0) = pose_variance_or_default(initial_guess.pose.covariance[0], 0.01);
        current_state_.sigma(1, 1) = pose_variance_or_default(initial_guess.pose.covariance[7], 0.01);
        current_state_.sigma(2, 2) = pose_variance_or_default(
            initial_guess.pose.covariance[35], params_.initial_yaw_covariance);
        current_state_.sigma(3, 3) = 0.002 * 0.002;  // v variance (2mm/s std)
        current_state_.sigma(4, 4) = (0.05 * M_PI / 180.0) * (0.05 * M_PI / 180.0);  // w variance (0.05°/s std)
        current_state_.sigma(5, 5) = params_.initial_gyro_bias_covariance;
        if (params_.enable_3d_estimation) {
            current_state_.sigma(6, 6) = pose_variance_or_default(
                initial_guess.pose.covariance[14], params_.initial_z_covariance);
            current_state_.sigma(7, 7) = pose_variance_or_default(
                initial_guess.pose.covariance[21], params_.initial_roll_pitch_covariance);
            current_state_.sigma(8, 8) = pose_variance_or_default(
                initial_guess.pose.covariance[28], params_.initial_roll_pitch_covariance);
        }
        current_state_.sigma(9, 9) = params_.initial_gyro_bias_covariance;
        current_state_.sigma(10, 10) = params_.initial_gyro_bias_covariance;

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
        wheel_odom_queue_.clear();
        imu_queue_.clear();
        first_sensor_time_aligned_ = false;
        has_wheel_measurement_ = false;
        gyro_bias_initialized_ =
            params_.load_initial_gyro_bias || !params_.gyro_initial_bias_enabled;
        last_received_wheel_stamp_.reset();
        last_received_imu_stamp_.reset();
        last_accepted_wheel_v_.reset();
        latest_wheel_angular_z_.reset();
        last_accepted_gyro_z_.reset();
        last_body_angular_velocity_ = geometry_msgs::msg::Vector3();
        stationary_start_time_.reset();
        stationary_gyro_sum_.setZero();
        stationary_gyro_samples_ = 0;
    }
    lm_covariance_history_.clear();
    landmark_initialized_ = false;

    const Eigen::Quaterniond angle(
        initial_guess.pose.pose.orientation.w,
        initial_guess.pose.pose.orientation.x,
        initial_guess.pose.pose.orientation.y,
        initial_guess.pose.pose.orientation.z);

    // Mark as initialized after finding the pose
    is_init_.store(true);

    Eigen::Matrix3d R = angle.toRotationMatrix();
    double yaw = std::atan2(R(1, 0), R(0, 0));
    logger_->info("Initial pose set at x={:.3f}, y={:.3f}, yaw={:.3f}°, gyro_bias=[{:.6f}, {:.6f}, {:.6f}]°/s",
                  initial_guess.pose.pose.position.x,
                  initial_guess.pose.pose.position.y,
                  yaw * 180.0 / M_PI,
                  applied_gyro_bias[0] * 180.0 / M_PI,
                  applied_gyro_bias[1] * 180.0 / M_PI,
                  applied_gyro_bias[2] * 180.0 / M_PI);
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
    }

    if (wheel_queue.empty() && imu_queue.empty()) {
        is_doing_odom_estimation_.store(false);
        return;
    }

    double last_v;
    geometry_msgs::msg::Vector3 last_body_angular_velocity;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_v = current_state_.mu(3);
    }
    {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        last_body_angular_velocity = last_body_angular_velocity_;
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
            if (!first_sensor_time_aligned_) {
                current_state_.time = measurement_time;
                first_sensor_time_aligned_ = true;
                return true;
            }
            state_time = current_state_.time;
        }

        const double dt = (measurement_time - state_time).seconds();
        if (dt < -1e-4) {
            logger_->warn("[processSensorFusionTick] Skipping stale measurement: dt={:.6f}", dt);
            return false;
        }

        if (dt > params_.max_prediction_dt) {
            logger_->warn(
                "[processSensorFusionTick] Large sensor dt {:.3f}s exceeds {:.3f}s; "
                "skipping propagation and realigning time",
                dt, params_.max_prediction_dt);
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_.time = measurement_time;
            return true;
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
        cmd_vel->angular = last_body_angular_velocity;

        if (use_wheel) {
            auto wheel_odom = wheel_queue.front();
            wheel_queue.pop_front();

            if (process_until(stamp_of_wheel(wheel_odom), cmd_vel)) {
                if (!correctionWheel(wheel_odom)) {
                    logger_->warn("[processSensorFusionTick] Wheel correction failed");
                } else {
                    last_v = wheel_odom->twist.twist.linear.x;
                }
            }
        } else {
            auto imu = imu_queue.front();
            imu_queue.pop_front();

            if (process_until(stamp_of_imu(imu), cmd_vel)) {
                if (!correctionImu(imu)) {
                    logger_->warn("[processSensorFusionTick] IMU correction failed");
                } else {
                    last_body_angular_velocity = imu->angular_velocity;
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        last_body_angular_velocity_ = last_body_angular_velocity;
    }

    auto result_odom = state2Msg();
    triggerNotify(ODOMETRY_UPDATED_EVENT, std::make_any<nav_msgs::msg::Odometry>(result_odom));

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
    triggerNotify(LANDMARK_PROCESSED_EVENT, std::make_any<nav_msgs::msg::Odometry>(result_odom));

    is_doing_odom_estimation_.store(false);
}

void OdomEstimatorEKF::processWheel(const nav_msgs::msg::Odometry::SharedPtr odom) // with update
{
    if (!odom || !std::isfinite(odom->twist.twist.linear.x)) {
        logger_->warn("[processWheel] Rejected non-finite wheel linear.x");
        return;
    }

    const rclcpp::Time stamp(odom->header.stamp);
    if (params_.reject_zero_timestamp && stamp.nanoseconds() == 0) {
        logger_->warn("[processWheel] Rejected zero timestamp");
        return;
    }

    if (!is_init_) {
        // Wheel pose/yaw is not a localization measurement. Initialize only an
        // identity local frame when no explicit initial pose was provided.
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header = odom->header;
        initial_pose.pose.pose.orientation.w = 1.0;
        findInitPose(initial_pose);
    }

    {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        if (last_received_wheel_stamp_ && stamp <= *last_received_wheel_stamp_) {
            logger_->warn("[processWheel] Rejected non-monotonic timestamp");
            return;
        }
        last_received_wheel_stamp_ = stamp;
        wheel_odom_queue_.push_back(odom);
        if (wheel_odom_queue_.size() > 2000) {
            wheel_odom_queue_.pop_front();
        }
    }

    logger_->debug("[processWheel] Stored latest wheel odometry: v={:.3f} m/s", odom->twist.twist.linear.x);
}

void OdomEstimatorEKF::processImu(const sensor_msgs::msg::Imu::SharedPtr imu) // with update
{
    if (!imu || !std::isfinite(imu->angular_velocity.x) ||
        !std::isfinite(imu->angular_velocity.y) ||
        !std::isfinite(imu->angular_velocity.z)) {
        logger_->warn("[processImu] Rejected non-finite base-frame angular velocity");
        return;
    }

    if (imu->header.frame_id != params_.imu_target_frame) {
        logger_->warn(
            "[processImu] Rejected IMU in frame '{}'; expected transformed frame '{}'",
            imu->header.frame_id, params_.imu_target_frame);
        return;
    }

    const rclcpp::Time stamp(imu->header.stamp);
    if (params_.reject_zero_timestamp && stamp.nanoseconds() == 0) {
        logger_->warn("[processImu] Rejected zero timestamp");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        if (last_received_imu_stamp_ && stamp <= *last_received_imu_stamp_) {
            logger_->warn("[processImu] Rejected non-monotonic timestamp");
            return;
        }
        last_received_imu_stamp_ = stamp;
        imu_queue_.push_back(imu);
        if (imu_queue_.size() > 10000) {
            imu_queue_.pop_front();
        }
    }

    logger_->debug(
        "[processImu] Stored latest base gyro: p={:.3f}, q={:.3f}, r={:.3f}°/s",
        imu->angular_velocity.x * 180.0 / M_PI,
        imu->angular_velocity.y * 180.0 / M_PI,
        imu->angular_velocity.z * 180.0 / M_PI);
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

    constexpr double kNumericalAngularRateThreshold = 1.0e-6;

    // Wheel speed is a body-X velocity. Angular velocity has already been
    // transformed into base_link and must be integrated as the complete body
    // rate [p, q, r], rather than treating r as Euler yaw rate.
    const double v = cmd_vel->linear.x;
    const double raw_p = cmd_vel->angular.x;
    const double raw_q = cmd_vel->angular.y;
    const double raw_r = cmd_vel->angular.z;
    const double gyro_bias_x = current_state_.mu(9);
    const double gyro_bias_y = current_state_.mu(10);
    const double gyro_bias_z = current_state_.mu(5);
    const double corrected_p = raw_p - gyro_bias_x;
    const double corrected_q = raw_q - gyro_bias_y;
    const double corrected_r = raw_r - gyro_bias_z;
    const double theta = current_state_.mu(2);
    const double roll = params_.enable_3d_estimation ? current_state_.mu(7) : 0.0;
    const double pitch = params_.enable_3d_estimation ? current_state_.mu(8) : 0.0;
    const Eigen::Vector3d corrected_body_rates(
        corrected_p, corrected_q, corrected_r);
    const std::array<double, 3> propagated_rpy = params_.enable_3d_estimation
        ? propagateRpyWithBodyRates(
            roll, pitch, theta, corrected_body_rates, dt)
        : std::array<double, 3>{
            0.0, 0.0, normalizeAngle(theta + corrected_r * dt)};
    const double propagated_roll = propagated_rpy[0];
    const double propagated_pitch = propagated_rpy[1];
    const double propagated_yaw = propagated_rpy[2];
    const double dtheta = normalizeAngle(propagated_yaw - theta);
    const double yaw_rate = dtheta / dt;
    const double cos_pitch = std::cos(pitch);
    const double sin_pitch = std::sin(pitch);
    const double horizontal_v = v * cos_pitch;
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    const int state_size = current_state_.mu.size();

    const auto apply_attitude_jacobian = [&](Eigen::MatrixXd& jacobian) {
        if (!params_.enable_3d_estimation) {
            jacobian(2, 4) = dt;
            jacobian(2, 5) = -dt;
            return;
        }

        const double sin_roll = std::sin(roll);
        const double cos_roll = std::cos(roll);
        const double safe_cos_pitch = std::copysign(
            std::max(std::fabs(cos_pitch), 1.0e-6), cos_pitch);
        const double tan_pitch = sin_pitch / safe_cos_pitch;
        const double sec_pitch_squared = 1.0 / (safe_cos_pitch * safe_cos_pitch);

        jacobian(7, 7) = 1.0 +
            (cos_roll * tan_pitch * corrected_q - sin_roll * tan_pitch * corrected_r) * dt;
        jacobian(7, 8) =
            (sin_roll * corrected_q + cos_roll * corrected_r) * sec_pitch_squared * dt;
        jacobian(8, 7) =
            (-sin_roll * corrected_q - cos_roll * corrected_r) * dt;
        jacobian(8, 8) = 1.0;
        jacobian(2, 7) =
            (cos_roll * corrected_q - sin_roll * corrected_r) / safe_cos_pitch * dt;
        jacobian(2, 8) = yaw_rate * tan_pitch * dt;

        const Eigen::Matrix3d body_to_euler =
            bodyRatesToEulerRatesMatrix(roll, pitch);
        constexpr std::array<int, 3> attitude_indices{7, 8, 2};
        constexpr std::array<int, 3> bias_indices{9, 10, 5};
        for (int row = 0; row < 3; ++row) {
            const int attitude_index =
                attitude_indices[static_cast<std::size_t>(row)];
            // Only raw body-z rate is retained as state 4.
            jacobian(attitude_index, 4) = body_to_euler(row, 2) * dt;
            for (int axis = 0; axis < 3; ++axis) {
                jacobian(
                    attitude_index,
                    bias_indices[static_cast<std::size_t>(axis)]) =
                    -body_to_euler(row, axis) * dt;
            }
        }
    };

    const auto add_attitude_process_noise = [&](Eigen::MatrixXd& process_noise) {
        if (!params_.enable_3d_estimation) {
            process_noise(2, 2) += params_.gyro_noise * dt * dt;
            return;
        }

        const Eigen::Matrix3d body_to_euler =
            bodyRatesToEulerRatesMatrix(roll, pitch);
        const Eigen::Matrix3d attitude_noise =
            body_to_euler *
            (Eigen::Matrix3d::Identity() * params_.gyro_noise * dt * dt) *
            body_to_euler.transpose();
        constexpr std::array<int, 3> attitude_indices{7, 8, 2};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                process_noise(
                    attitude_indices[static_cast<std::size_t>(row)],
                    attitude_indices[static_cast<std::size_t>(column)]) +=
                    attitude_noise(row, column);
            }
        }
        process_noise(7, 7) += params_.roll_pitch_process_noise * dt;
        process_noise(8, 8) += params_.roll_pitch_process_noise * dt;
    };

    // Determine if this is a spin turn (in-place rotation) or normal Ackermann drive
    // Spin turn: very low linear velocity but significant angular velocity
    if (std::fabs(v) < params_.wheel_stationary_threshold &&
        std::fabs(yaw_rate) > kNumericalAngularRateThreshold) {
        // === Spin Turn Mode ===
        // Robot rotates in place around its center
        // Use angular velocity directly
        // For spin turn, position change is minimal (rotating in place)
        dx = 0.0;
        dy = 0.0;
        dz = 0.0;

        logger_->debug("[prediction] Spin turn mode: body_rates=[{:.3f}, {:.3f}, {:.3f}]°/s bias_z={:.3f}°/s yaw_rate={:.3f}°/s dθ={:.4f}° dt={:.4f}",
                       raw_p * 180.0 / M_PI, raw_q * 180.0 / M_PI, raw_r * 180.0 / M_PI,
                       gyro_bias_z * 180.0 / M_PI, yaw_rate * 180.0 / M_PI,
                       dtheta * 180.0 / M_PI, dt);

        // Jacobians for spin turn
        Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(state_size, state_size);
        // For pure rotation, position doesn't change with theta
        Gt(0, 2) = 0.0;
        Gt(1, 2) = 0.0;
        // Position doesn't depend on velocities in spin turn
        Gt(0, 3) = 0.0;  // ∂x/∂v = 0 (no translation)
        Gt(1, 3) = 0.0;  // ∂y/∂v = 0 (no translation)
        Gt(0, 4) = 0.0;  // ∂x/∂w = 0 (no translation)
        Gt(1, 4) = 0.0;  // ∂y/∂w = 0 (no translation)
        apply_attitude_jacobian(Gt);
        // Velocities remain as identity (v_new = v_old, w_new = w_old)

        // Discrete process noise. Gyro white noise integrates into yaw with
        // dt^2, while bias random walk grows with elapsed time.
        Eigen::MatrixXd Q_process = Eigen::MatrixXd::Zero(state_size, state_size);
        const double position_variance = params_.wheel_velocity_noise * dt * dt;
        Q_process(0, 0) = position_variance;
        Q_process(1, 1) = position_variance;
        Q_process(3, 3) = params_.wheel_velocity_noise * dt;
        Q_process(4, 4) = params_.gyro_noise * dt;
        Q_process(5, 5) = params_.gyro_bias_random_walk * dt;
        if (params_.enable_3d_estimation) {
            Q_process(9, 9) = params_.gyro_bias_random_walk * dt;
            Q_process(10, 10) = params_.gyro_bias_random_walk * dt;
            Q_process(6, 6) = params_.vertical_motion_noise * dt * dt;
        }
        add_attitude_process_noise(Q_process);
        // Landmarks don't have process noise

        // 공분산 예측
        current_state_.sigma = Gt * current_state_.sigma * Gt.transpose() + Q_process;
    } else {
        // === Normal Ackermann Drive Mode ===
        // motion model

        if (std::fabs(yaw_rate) > kNumericalAngularRateThreshold) {
            // 회전 주행 (curved motion)
            const double R = horizontal_v / yaw_rate;
            dx = R * (sin(theta + dtheta) - sin(theta));
            dy = -R * (cos(theta + dtheta) - cos(theta));
        } else {
            // 직선 주행 (straight line motion)
            dx = horizontal_v * cos(theta) * dt;
            dy = horizontal_v * sin(theta) * dt;
        }
        // REP-103 positive pitch rotates body +X toward world -Z. Therefore a
        // nose-up slope has negative pitch and produces positive dz.
        if (params_.enable_3d_estimation) {
            dz = projectForwardVelocity(v, pitch, theta).z() * dt;
        }

        logger_->debug("[prediction] Ackermann mode: v={:.3f} body_rates=[{:.3f}, {:.3f}, {:.3f}]°/s bias_z={:.3f}°/s yaw_rate={:.3f}°/s dθ={:.4f}° dt={:.4f}",
                       v, raw_p * 180.0 / M_PI, raw_q * 180.0 / M_PI,
                       raw_r * 180.0 / M_PI, gyro_bias_z * 180.0 / M_PI,
                       yaw_rate * 180.0 / M_PI, dtheta * 180.0 / M_PI, dt);

        // Jacobians for Ackermann steering - must match state size for SLAM mode
        Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(state_size, state_size);

        // Position derivatives w.r.t. theta
        Gt(0, 2) = -horizontal_v * dt * sin(theta);
        Gt(1, 2) =  horizontal_v * dt * cos(theta);

        // Position derivatives w.r.t. v (linear velocity)
        Gt(0, 3) = cos_pitch * dt * cos(theta);
        Gt(1, 3) = cos_pitch * dt * sin(theta);
        if (params_.enable_3d_estimation) {
            Gt(6, 3) = -sin_pitch * dt;
            Gt(6, 8) = -v * cos_pitch * dt;
        }

        // Position and orientation derivatives w.r.t. w (angular velocity)
        if (std::fabs(yaw_rate) > kNumericalAngularRateThreshold) {
            // Curved motion
            const double R = horizontal_v / yaw_rate;
            const double dx_d_yaw_rate =
                dt * R * cos(theta + dtheta) -
                (horizontal_v / (yaw_rate * yaw_rate)) *
                (sin(theta + dtheta) - sin(theta));
            const double dy_d_yaw_rate =
                dt * R * sin(theta + dtheta) +
                (horizontal_v / (yaw_rate * yaw_rate)) *
                (cos(theta + dtheta) - cos(theta));
            const double yaw_rate_from_raw_r =
                bodyRatesToEulerRatesMatrix(roll, pitch)(2, 2);
            Gt(0, 4) = dx_d_yaw_rate * yaw_rate_from_raw_r;
            Gt(1, 4) = dy_d_yaw_rate * yaw_rate_from_raw_r;
            Gt(0, 5) = -Gt(0, 4);
            Gt(1, 5) = -Gt(1, 4);
        } else {
            // Straight motion
            Gt(0, 4) = 0.0;
            Gt(1, 4) = 0.0;
            Gt(0, 5) = 0.0;
            Gt(1, 5) = 0.0;
        }
        apply_attitude_jacobian(Gt);
        if (params_.enable_3d_estimation && std::fabs(cos_pitch) > 1.0e-6) {
            Gt(0, 8) = -dx * std::tan(pitch);
            Gt(1, 8) = -dy * std::tan(pitch);
        }

        // Forward wheel noise and soft lateral-slip uncertainty are rotated
        // from the body frame into odom. This keeps vy=0 as the model mean,
        // without pretending lateral motion is impossible.
        Eigen::MatrixXd Q_process = Eigen::MatrixXd::Zero(state_size, state_size);
        const double forward_variance = params_.wheel_velocity_noise * dt * dt;
        const double lateral_variance = params_.vy_zero_constraint_enabled
            ? params_.vy_zero_constraint_noise * dt * dt
            : params_.vy_zero_constraint_noise * dt;
        const double cos_theta = std::cos(theta);
        const double sin_theta = std::sin(theta);
        Q_process(0, 0) =
            cos_theta * cos_theta * forward_variance +
            sin_theta * sin_theta * lateral_variance;
        Q_process(1, 1) =
            sin_theta * sin_theta * forward_variance +
            cos_theta * cos_theta * lateral_variance;
        Q_process(0, 1) =
            sin_theta * cos_theta * (forward_variance - lateral_variance);
        Q_process(1, 0) = Q_process(0, 1);
        Q_process(3, 3) = params_.wheel_velocity_noise * dt;
        Q_process(4, 4) = params_.gyro_noise * dt;
        Q_process(5, 5) = params_.gyro_bias_random_walk * dt;
        if (params_.enable_3d_estimation) {
            Q_process(9, 9) = params_.gyro_bias_random_walk * dt;
            Q_process(10, 10) = params_.gyro_bias_random_walk * dt;
            Q_process(6, 6) = params_.vertical_motion_noise * dt * dt;
        }
        add_attitude_process_noise(Q_process);
        // Landmarks don't have process noise

        // 공분산 예측
        current_state_.sigma = Gt * current_state_.sigma * Gt.transpose() + Q_process;
    }

    // 상태 예측 (공통)
    current_state_.mu(0) += dx;
    current_state_.mu(1) += dy;
    current_state_.mu(6) = params_.enable_3d_estimation
        ? current_state_.mu(6) + dz : 0.0;
    current_state_.mu(2) = propagated_yaw;
    current_state_.mu(7) = params_.enable_3d_estimation ? propagated_roll : 0.0;
    current_state_.mu(8) = params_.enable_3d_estimation ? propagated_pitch : 0.0;
    current_state_.mu(3) = v;   // Update velocity
    current_state_.mu(4) = raw_r;  // Raw body-z rate; prediction removes z-bias.

    logger_->debug("[prediction] Predicted state: x={:.3f}, y={:.3f}, z={:.3f}, "
                  "roll={:.2f}°, pitch={:.2f}°, yaw={:.3f}°, v={:.3f}, "
                  "raw_w={:.3f}°/s, bias={:.3f}°/s",
                  current_state_.mu(0),
                  current_state_.mu(1),
                  current_state_.mu(6),
                  current_state_.mu(7) * 180.0 / M_PI,
                  current_state_.mu(8) * 180.0 / M_PI,
                  current_state_.mu(2) * 180.0 / M_PI,
                  current_state_.mu(3),
                  current_state_.mu(4) * 180.0 / M_PI,
                  current_state_.mu(5) * 180.0 / M_PI);

    current_state_.mu(2) = normalizeAngle(current_state_.mu(2));
    current_state_.mu(7) = normalizeAngle(current_state_.mu(7));
    current_state_.mu(8) = std::clamp(
        current_state_.mu(8), -M_PI_2 + 1.0e-6, M_PI_2 - 1.0e-6);

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
    int robot_state_size = State::getRobotStateSize();

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

    // V-marker observation is explicitly planar (x, y, yaw). It must not
    // directly pull relative height or gravity-referenced roll/pitch.
    K.row(6).setZero();
    K.row(7).setZero();
    K.row(8).setZero();

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

    const auto update_single_state = [this](
        int state_index,
        double innovation,
        double measurement_variance) {
        const int state_size = current_state_.mu.size();
        const double innovation_variance =
            current_state_.sigma(state_index, state_index) + measurement_variance;
        Eigen::MatrixXd observation = Eigen::MatrixXd::Zero(1, state_size);
        observation(0, state_index) = 1.0;
        Eigen::VectorXd gain = Eigen::VectorXd::Zero(state_size);
        gain(state_index) =
            current_state_.sigma(state_index, state_index) / innovation_variance;
        current_state_.mu += gain * innovation;
        const Eigen::MatrixXd identity =
            Eigen::MatrixXd::Identity(state_size, state_size);
        const Eigen::MatrixXd residual_transform = identity - gain * observation;
        current_state_.sigma =
            residual_transform * current_state_.sigma * residual_transform.transpose() +
            gain * measurement_variance * gain.transpose();
    };

    const Eigen::Vector3d measured_rates(
        imu->angular_velocity.x,
        imu->angular_velocity.y,
        imu->angular_velocity.z);
    const std::array<double, 3> measurement_variances{
        covarianceOrFallback(
            imu->angular_velocity_covariance[0],
            params_.use_imu_message_covariance,
            params_.gyro_noise,
            params_.gyro_min_variance),
        covarianceOrFallback(
            imu->angular_velocity_covariance[4],
            params_.use_imu_message_covariance,
            params_.gyro_noise,
            params_.gyro_min_variance),
        covarianceOrFallback(
            imu->angular_velocity_covariance[8],
            params_.use_imu_message_covariance,
            params_.gyro_noise,
            params_.gyro_min_variance)};
    const double measured_w = measured_rates.z();

    if (!last_accepted_gyro_z_) {
        current_state_.mu(4) = measured_w;
        current_state_.sigma(4, 4) = measurement_variances[2];
    } else {
        const double innovation = measured_w - current_state_.mu(4);
        update_single_state(4, innovation, measurement_variances[2]);
    }
    last_accepted_gyro_z_ = measured_w;

    // sensor_msgs/Imu.orientation is intentionally ignored on every axis.
    // Attitude comes only from the joint body-rate quaternion propagation.
    const double gyro_magnitude = measured_rates.norm();
    const bool stationary =
        has_wheel_measurement_ &&
        latest_wheel_angular_z_ &&
        std::fabs(current_state_.mu(3)) < params_.wheel_stationary_threshold &&
        std::fabs(*latest_wheel_angular_z_) <
            params_.wheel_stationary_angular_threshold &&
        gyro_magnitude < params_.gyro_stationary_threshold;

    if (!stationary) {
        stationary_start_time_.reset();
        stationary_gyro_sum_.setZero();
        stationary_gyro_samples_ = 0;
    } else {
        const rclcpp::Time stamp(imu->header.stamp);
        if (!stationary_start_time_) {
            stationary_start_time_ = stamp;
        }
        stationary_gyro_sum_ += measured_rates;
        ++stationary_gyro_samples_;
        const double stationary_duration =
            (stamp - *stationary_start_time_).seconds();

        if (!gyro_bias_initialized_ && params_.gyro_initial_bias_enabled &&
            stationary_duration >= params_.gyro_initial_bias_duration &&
            stationary_gyro_samples_ >=
                static_cast<std::size_t>(params_.gyro_initial_bias_min_samples)) {
            const Eigen::Vector3d calibrated_bias =
                stationary_gyro_sum_ /
                static_cast<double>(stationary_gyro_samples_);
            constexpr std::array<int, 3> bias_indices{9, 10, 5};
            for (int axis = 0; axis < 3; ++axis) {
                const int bias_index =
                    bias_indices[static_cast<std::size_t>(axis)];
                current_state_.mu(bias_index) = calibrated_bias(axis);
                const double calibrated_variance = std::max(
                    params_.gyro_min_variance,
                    measurement_variances[static_cast<std::size_t>(axis)] /
                        static_cast<double>(stationary_gyro_samples_));
                current_state_.sigma.row(bias_index).setZero();
                current_state_.sigma.col(bias_index).setZero();
                current_state_.sigma(bias_index, bias_index) =
                    calibrated_variance;
            }
            gyro_bias_initialized_ = true;
            logger_->info(
                "Initial stationary gyro bias calibrated: [{:.8f}, {:.8f}, {:.8f}] rad/s from {} samples",
                current_state_.mu(9), current_state_.mu(10), current_state_.mu(5),
                stationary_gyro_samples_);
        } else if (gyro_bias_initialized_ &&
                   stationary_duration >= params_.gyro_initial_bias_duration) {
            // Zero-angular-rate update: gyro_xyz = bias_xyz + noise.
            constexpr std::array<int, 3> bias_indices{9, 10, 5};
            for (int axis = 0; axis < 3; ++axis) {
                const int bias_index =
                    bias_indices[static_cast<std::size_t>(axis)];
                update_single_state(
                    bias_index,
                    measured_rates(axis) - current_state_.mu(bias_index),
                    params_.stationary_gyro_bias_variance);
            }
        }
    }

    logger_->debug(
        "[correctionImu] base gyro=[{:.5f}, {:.5f}, {:.5f}], bias=[{:.5f}, {:.5f}, {:.5f}] rad/s, roll={:.2f} deg, pitch={:.2f} deg",
        measured_rates.x(), measured_rates.y(), measured_rates.z(),
        current_state_.mu(9), current_state_.mu(10), current_state_.mu(5),
        current_state_.mu(7) * 180.0 / M_PI,
        current_state_.mu(8) * 180.0 / M_PI);

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
    R_subset(0, 0) = covarianceOrFallback(
        odom->twist.covariance[0],
        params_.use_wheel_message_covariance,
        params_.wheel_velocity_noise,
        params_.wheel_min_velocity_variance);

    logger_->debug("[correctionWheel] Measured v: {:.3f} m/s, Predicted v: {:.3f} m/s, Variance: {:.6e} (std: {:.3f} m/s)",
                  measured_v, h_mu(0), R_subset(0, 0), std::sqrt(R_subset(0, 0)));

    if (!last_accepted_wheel_v_) {
        current_state_.mu(3) = measured_v;
        current_state_.sigma(3, 3) = R_subset(0, 0);
        last_accepted_wheel_v_ = measured_v;
        if (std::isfinite(odom->twist.twist.angular.z)) {
            latest_wheel_angular_z_ = odom->twist.twist.angular.z;
        } else {
            latest_wheel_angular_z_.reset();
        }
        has_wheel_measurement_ = true;
        return true;
    }

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
    K(6) = 0.0;
    K(7) = 0.0;
    K(8) = 0.0;
    K(9) = 0.0;
    K(10) = 0.0;

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
    last_accepted_wheel_v_ = measured_v;
    if (std::isfinite(odom->twist.twist.angular.z)) {
        latest_wheel_angular_z_ = odom->twist.twist.angular.z;
    } else {
        latest_wheel_angular_z_.reset();
    }
    has_wheel_measurement_ = true;

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
    // Landmark state is only meaningful while landmark correction is enabled.
    if (!is_landmark_correction_enabled_.load()) {
        return false;
    }

    return landmark_initialized_;
}


} // namespace odom_estimation
} // namespace gole
