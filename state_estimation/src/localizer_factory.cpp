#include <stdexcept>
#include <algorithm>

#include "logger/logger.hpp"

#include "localization/localizer_factory.hpp"
#include "localization/localizer/localizer_ekf.hpp"

namespace gole {
namespace localization {

std::unique_ptr<LocalizerBase> LocalizerFactory::createLocalizer(
    const std::string& localizer_type, 
    rclcpp_lifecycle::LifecycleNode* node,
    std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager,
    const Params& params) 
{
    if (localizer_type == "EKF") {
        auto localizer = std::make_unique<LocalizerEKF>(node, logger, diagnostic_manager, params);
        return localizer;
    // } else if (localizer_type == "AMCL") {
    //     logger->info("Initializing AMCL localizer");
    //     auto localizer = std::make_unique<AmclLocalizer>(node);
    //     logger->info("AMCL localizer created successfully");
    //     return localizer;
    } else {
        throw std::runtime_error("Unknown localizer type: " + localizer_type + ". Valid options: 'EKF', 'AMCL'");
    }
}

} // namespace localization
} // namespace gole