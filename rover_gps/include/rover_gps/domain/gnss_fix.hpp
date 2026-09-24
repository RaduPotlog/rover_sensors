// Copyright 2026 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROVER_GPS_DOMAIN_GNSS_FIX_HPP_
#define ROVER_GPS_DOMAIN_GNSS_FIX_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace rover_gps::domain
{

/** @brief Fix quality, in the order of sensor_msgs/NavSatStatus. */
enum class FixStatus
{
    NoFix,
    Fix,
    SbasFix,
    GbasFix,
};

inline bool hasFix(FixStatus status)
{
    return status != FixStatus::NoFix;
}

/** @brief One GNSS position sample. */
struct GnssFix
{
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
    FixStatus status{FixStatus::NoFix};
    // Horizontal 1-sigma position error [m]; NaN when the receiver gives no estimate.
    double horizontal_std_m{std::numeric_limits<double>::quiet_NaN()};
    // Time the fix was received [s].
    double stamp_s{0.0};
};

/**
 * @brief Horizontal 1-sigma position error [m] from the east and north position variances [m^2].
 * @details The worse of the two axes, so the health thresholds (GpsHealthEvaluator) judge the fix
 *          by its weakest direction. NaN when that variance is negative, i.e. not a real estimate.
 */
inline double horizontalStdFromVariances(double east_variance_m2, double north_variance_m2)
{
    const double variance = std::max(east_variance_m2, north_variance_m2);
    return variance >= 0.0 ? std::sqrt(variance) : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace rover_gps::domain

#endif  // ROVER_GPS_DOMAIN_GNSS_FIX_HPP_
