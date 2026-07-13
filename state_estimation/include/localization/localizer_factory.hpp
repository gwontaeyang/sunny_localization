#ifndef LOCALIZATION_LOCALIZER_LOCALIZER_FACTORY_HPP_
#define LOCALIZATION_LOCALIZER_LOCALIZER_FACTORY_HPP_

#include <memory>

#include "localization/localizer/localizer_base.hpp"
#include "diagnostic/diagnostic_manager.hpp"
#include "logger/logger.hpp"

namespace gole {
namespace localization {

/**
 * @brief Localizer factory for creating appropriate localizer instance
 */
class LocalizerFactory {
public:
    static std::unique_ptr<LocalizerBase> createLocalizer(
        const std::string& localizer_type, 
        rclcpp_lifecycle::LifecycleNode* node,
        std::shared_ptr<spdlog::logger> logger,
        std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager,
        const Params& params);
};

} // namespace localization
} // namespace gole

#endif // LOCALIZATION_LOCALIZER_LOCALIZER_FACTORY_HPP_