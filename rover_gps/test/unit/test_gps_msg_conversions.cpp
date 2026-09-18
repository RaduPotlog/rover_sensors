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

#include <gtest/gtest.h>

#include <cmath>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

#include "rover_gps/infrastructure/gps_msg_conversions.hpp"

using namespace rover_gps::infrastructure;  // NOLINT
using rover_gps::domain::FixStatus;
using rover_gps::domain::HealthLevel;
using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;
using NavSatStatusMsg = sensor_msgs::msg::NavSatStatus;

TEST(GpsMsgConversionsTest, NavSatFixToGnssFix)
{
    NavSatFixMsg msg;
    msg.latitude = 45.5;
    msg.longitude = 25.25;
    msg.altitude = 300.0;
    msg.status.status = NavSatStatusMsg::STATUS_SBAS_FIX;
    msg.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_APPROXIMATED;
    msg.position_covariance[0] = 4.0;
    msg.position_covariance[4] = 9.0;

    const auto fix = toGnssFix(msg, 12.5);
    EXPECT_DOUBLE_EQ(fix.latitude_deg, 45.5);
    EXPECT_DOUBLE_EQ(fix.longitude_deg, 25.25);
    EXPECT_DOUBLE_EQ(fix.altitude_m, 300.0);
    EXPECT_EQ(fix.status, FixStatus::SbasFix);
    EXPECT_DOUBLE_EQ(fix.horizontal_std_m, 3.0);
    EXPECT_DOUBLE_EQ(fix.stamp_s, 12.5);
}

TEST(GpsMsgConversionsTest, UnknownCovarianceAndNoFix)
{
    NavSatFixMsg msg;
    msg.status.status = NavSatStatusMsg::STATUS_NO_FIX;
    msg.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_UNKNOWN;

    const auto fix = toGnssFix(msg, 0.0);
    EXPECT_EQ(fix.status, FixStatus::NoFix);
    EXPECT_TRUE(std::isnan(fix.horizontal_std_m));
}

TEST(GpsMsgConversionsTest, DiagnosticLevels)
{
    EXPECT_EQ(toDiagnosticLevel(HealthLevel::Ok), DiagnosticStatusMsg::OK);
    EXPECT_EQ(toDiagnosticLevel(HealthLevel::Warn), DiagnosticStatusMsg::WARN);
    EXPECT_EQ(toDiagnosticLevel(HealthLevel::Error), DiagnosticStatusMsg::ERROR);
    EXPECT_EQ(toDiagnosticLevel(HealthLevel::Stale), DiagnosticStatusMsg::STALE);
}
