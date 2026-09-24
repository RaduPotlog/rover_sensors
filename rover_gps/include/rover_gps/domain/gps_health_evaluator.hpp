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

#ifndef ROVER_GPS_DOMAIN_GPS_HEALTH_EVALUATOR_HPP_
#define ROVER_GPS_DOMAIN_GPS_HEALTH_EVALUATOR_HPP_

#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include "rover_gps/domain/gnss_fix.hpp"

namespace rover_gps::domain
{

enum class HealthLevel
{
    Ok,
    Warn,
    Error,
    Stale,
};

struct GpsHealthThresholds
{
    double expected_rate_hz{1.0};
    // Rate below expected_rate_hz * min_rate_ratio is reported as WARN.
    double min_rate_ratio{0.5};
    // No fix message for this long is an ERROR (NMEA link lost), a WARN without gps_required.
    double fix_timeout_s{3.0};
    double warn_horizontal_std_m{5.0};
    double error_horizontal_std_m{20.0};
    // False when nothing localizes on GPS (indoors: ROVER_LOCALIZATION_SOURCE indoor, slam or
    // amcl). Poor accuracy or a GPS that is switched off is then not a fault, so neither missing
    // data nor a timeout nor low accuracy goes beyond WARN.
    bool gps_required{true};
};

struct GpsHealthReport
{
    HealthLevel level{HealthLevel::Stale};
    std::string message;
    // Everything below is only meaningful when a fix has been received.
    bool has_data{false};
    GnssFix last_fix;
    double rate_hz{0.0};
    double age_s{0.0};
    std::uint64_t fix_count{0};
};

/**
 * @brief Classifies the GNSS stream: link alive, fix quality, accuracy and message rate.
 * @details Mirrors rover_rs16_lidar::domain::LidarHealthEvaluator (same HealthLevel values, same
 *          rate window and timeout logic); a change to one should be checked against the other.
 *          Sharing the code waits for a third sensor. One deliberate difference: only the lidar
 *          has a startup grace that turns a stream never seen into an ERROR.
 */
class GpsHealthEvaluator
{
public:
    explicit GpsHealthEvaluator(GpsHealthThresholds thresholds);

    /** @throws std::invalid_argument when a threshold is not positive or warn > error. */
    static void validate(const GpsHealthThresholds & thresholds);

    void addFix(const GnssFix & fix);

    GpsHealthReport evaluate(double now_s) const;

private:
    double rateHz(double now_s) const;

    GpsHealthThresholds thresholds_;
    std::optional<GnssFix> last_fix_;
    std::deque<double> arrival_times_s_;
    std::uint64_t fix_count_{0};
};

}  // namespace rover_gps::domain

#endif  // ROVER_GPS_DOMAIN_GPS_HEALTH_EVALUATOR_HPP_
