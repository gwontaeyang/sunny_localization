#ifndef ODOM_ESTIMATION_ODOM_ESTIMATOR_FACTORY_HPP_
#define ODOM_ESTIMATION_ODOM_ESTIMATOR_FACTORY_HPP_

#include <memory>

#include "odom_estimation/odom_estimator/odom_estimator_base.hpp"
#include "diagnostic/diagnostic_manager.hpp"
#include "logger/logger.hpp"

namespace gole {
namespace odom_estimation {

/**
 * @brief OdomEstimator factory for creating appropriate odom_estimator instance
 */
class OdomEstimatorFactory {
public:
    static std::unique_ptr<OdomEstimatorBase> createOdomEstimator(
        const std::string& odom_estimator_type, 
        rclcpp_lifecycle::LifecycleNode* node,
        std::shared_ptr<spdlog::logger> logger,
        std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager,
        const Params& params);
};

} // namespace odom_estimation
} // namespace gole

#endif // ODOM_ESTIMATION_ODOM_ESTIMATOR_FACTORY_HPP_
