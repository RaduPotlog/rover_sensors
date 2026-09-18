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

#ifndef ROVER_GPS_INFRASTRUCTURE_GPS_MSG_CONVERSIONS_HPP_
#define ROVER_GPS_INFRASTRUCTURE_GPS_MSG_CONVERSIONS_HPP_

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rover_gps/domain/gnss_fix.hpp"
#include "rover_gps/domain/gps_health_evaluator.hpp"

namespace rover_gps::infrastructure
{

using NavSatFixMsg = sensor_msgs::msg::NavSatFix;

/** @param stamp_s receive time [s]. */
domain::GnssFix toGnssFix(const NavSatFixMsg & msg, double stamp_s);

unsigned char toDiagnosticLevel(domain::HealthLevel level);

const char * fixStatusText(domain::FixStatus status);

}  // namespace rover_gps::infrastructure

#endif  // ROVER_GPS_INFRASTRUCTURE_GPS_MSG_CONVERSIONS_HPP_
