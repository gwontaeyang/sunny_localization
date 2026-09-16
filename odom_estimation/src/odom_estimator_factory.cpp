#include <stdexcept>

#include "odom_estimation/odom_estimator_factory.hpp"
#include "odom_estimation/odom_estimator/odom_estimator_ekf.hpp"

namespace gole {
namespace odom_estimation {

std::unique_ptr<OdomEstimatorBase> OdomEstimatorFactory::createOdomEstimator(
    const std::string& odom_estimator_type, 
    rclcpp_lifecycle::LifecycleNode* node,
    std::shared_ptr<spdlog::logger> logger,
    const Params& params) 
{
    if (odom_estimator_type == "EKF") {
        return std::make_unique<OdomEstimatorEKF>(node, logger, params);
    } else {
        throw std::runtime_error(
            "Unknown odom_estimator type: " + odom_estimator_type +
            ". The only implemented option is 'EKF'");
    }
}

} // namespace odom_estimation
} // namespace gole
