#include <stdexcept>
#include <algorithm>

#include "logger/logger.hpp"

#include "odom_estimation/odom_estimator_factory.hpp"
#include "odom_estimation/odom_estimator/odom_estimator_ekf.hpp"

namespace gole {
namespace odom_estimation {

std::unique_ptr<OdomEstimatorBase> OdomEstimatorFactory::createOdomEstimator(
    const std::string& odom_estimator_type, 
    rclcpp_lifecycle::LifecycleNode* node,
    std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager,
    const Params& params) 
{
    if (odom_estimator_type == "EKF") {
        auto odom_estimator = std::make_unique<OdomEstimatorEKF>(node, logger, diagnostic_manager, params);
        return odom_estimator;
    // } else if (odom_estimator_type == "AMCL") {
    //     logger->info("Initializing AMCL odom_estimator");
    //     auto odom_estimator = std::make_unique<AmclOdomEstimator>(node);
    //     logger->info("AMCL odom_estimator created successfully");
    //     return odom_estimator;
    } else {
        throw std::runtime_error("Unknown odom_estimator type: " + odom_estimator_type + ". Valid options: 'EKF', 'AMCL'");
    }
}

} // namespace odom_estimation
} // namespace gole