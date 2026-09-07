#ifndef ODOM_ESTIMATION_MEASUREMENT_VALIDATION_HPP_
#define ODOM_ESTIMATION_MEASUREMENT_VALIDATION_HPP_

#include <algorithm>
#include <cmath>

namespace gole {
namespace odom_estimation {

inline double covarianceOrFallback(
    double message_variance,
    bool use_message_covariance,
    double fallback_variance,
    double minimum_variance)
{
    const bool valid_message_variance =
        std::isfinite(message_variance) && message_variance > 0.0;
    const double selected = use_message_covariance && valid_message_variance
        ? message_variance
        : fallback_variance;
    return std::max(selected, minimum_variance);
}

}  // namespace odom_estimation
}  // namespace gole

#endif  // ODOM_ESTIMATION_MEASUREMENT_VALIDATION_HPP_
