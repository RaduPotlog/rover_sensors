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

#include "rover_gps/infrastructure/gps_msg_conversions.hpp"

#include <cstdint>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace rover_gps::infrastructure
{

namespace
{
using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;
using NavSatStatusMsg = sensor_msgs::msg::NavSatStatus;

domain::FixStatus toFixStatus(std::int8_t status)
{
    switch (status) {
        case NavSatStatusMsg::STATUS_FIX:
            return domain::FixStatus::Fix;
        case NavSatStatusMsg::STATUS_SBAS_FIX:
            return domain::FixStatus::SbasFix;
        case NavSatStatusMsg::STATUS_GBAS_FIX:
            return domain::FixStatus::GbasFix;
        default:
            return domain::FixStatus::NoFix;
    }
}
}  // namespace

domain::GnssFix toGnssFix(const NavSatFixMsg & msg, double stamp_s)
{
    domain::GnssFix fix;
    fix.latitude_deg = msg.latitude;
    fix.longitude_deg = msg.longitude;
    fix.altitude_m = msg.altitude;
    fix.status = toFixStatus(msg.status.status);
    fix.stamp_s = stamp_s;

    if (msg.position_covariance_type != NavSatFixMsg::COVARIANCE_TYPE_UNKNOWN) {
        // Row-major 3x3 (ENU): [0] is the east variance, [4] the north variance.
        fix.horizontal_std_m = domain::horizontalStdFromVariances(
            msg.position_covariance[0], msg.position_covariance[4]);
    }
    return fix;
}

unsigned char toDiagnosticLevel(domain::HealthLevel level)
{
    switch (level) {
        case domain::HealthLevel::Ok:
            return DiagnosticStatusMsg::OK;
        case domain::HealthLevel::Warn:
            return DiagnosticStatusMsg::WARN;
        case domain::HealthLevel::Error:
            return DiagnosticStatusMsg::ERROR;
        default:
            return DiagnosticStatusMsg::STALE;
    }
}

const char * fixStatusText(domain::FixStatus status)
{
    switch (status) {
        case domain::FixStatus::Fix:
            return "Fix";
        case domain::FixStatus::SbasFix:
            return "SBAS fix";
        case domain::FixStatus::GbasFix:
            return "GBAS fix";
        default:
            return "No fix";
    }
}

}  // namespace rover_gps::infrastructure
