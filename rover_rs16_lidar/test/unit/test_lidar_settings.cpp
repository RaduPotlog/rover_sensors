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

#include <limits>
#include <stdexcept>
#include <string>

#include "rover_rs16_lidar/domain/lidar_settings.hpp"

namespace
{

using rover_rs16_lidar::domain::LidarInputType;
using rover_rs16_lidar::domain::LidarSettings;
using rover_rs16_lidar::domain::parseInputType;
using rover_rs16_lidar::domain::SelfFilterBox;

/// Defaults are what config/rover_rs16_lidar.yaml ships, so they must pass validation.
LidarSettings validSettings()
{
    return LidarSettings{};
}

SelfFilterBox validBox(const std::string & name)
{
    SelfFilterBox box;
    box.name = name;
    box.min_x = -0.10;
    box.max_x = 0.08;
    box.min_y = 0.33;
    box.max_y = 0.47;
    box.min_z = -0.25;
    box.max_z = 0.25;
    return box;
}

}  // namespace

TEST(ParseInputType, AcceptsTheTwoSupportedSources)
{
    EXPECT_EQ(parseInputType("lidar"), LidarInputType::Lidar);
    EXPECT_EQ(parseInputType("pcap"), LidarInputType::Pcap);
}

TEST(ParseInputType, RejectsAnythingElse)
{
    EXPECT_THROW(parseInputType(""), std::invalid_argument);
    EXPECT_THROW(parseInputType("LIDAR"), std::invalid_argument);
    EXPECT_THROW(parseInputType("rosbag"), std::invalid_argument);
}

TEST(LidarSettingsValidate, AcceptsTheShippedDefaults)
{
    EXPECT_NO_THROW(LidarSettings::validate(validSettings()));
}

TEST(LidarSettingsValidate, RejectsEmptyFrameOrTopics)
{
    auto settings = validSettings();
    settings.frame_id.clear();
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings = validSettings();
    settings.point_cloud_topic.clear();
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings = validSettings();
    settings.scan_topic.clear();
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);
}

TEST(LidarSettingsValidate, IgnoresScanFieldsWhenTheScanIsOff)
{
    auto settings = validSettings();
    settings.scan.enabled = false;
    settings.scan_topic.clear();
    settings.scan.angle_increment = -1.0;
    EXPECT_NO_THROW(LidarSettings::validate(settings));
}

TEST(LidarSettingsValidate, RejectsUnusablePorts)
{
    auto settings = validSettings();
    settings.sensor.msop_port = 0;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings = validSettings();
    settings.sensor.difop_port = settings.sensor.msop_port;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);
}

TEST(LidarSettingsValidate, RejectsInvertedDistanceRange)
{
    auto settings = validSettings();
    settings.sensor.min_distance = settings.sensor.max_distance;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings = validSettings();
    settings.sensor.min_distance = -1.0F;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);
}

TEST(LidarSettingsValidate, RejectsAnglesOutsideAFullRevolution)
{
    auto settings = validSettings();
    settings.sensor.start_angle = 400.0F;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings = validSettings();
    settings.sensor.end_angle = -1.0F;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);
}

TEST(LidarSettingsValidate, RequiresAPathWhenReplayingACapture)
{
    auto settings = validSettings();
    settings.sensor.input_type = LidarInputType::Pcap;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings.sensor.pcap_path = "/tmp/rs16.pcap";
    EXPECT_NO_THROW(LidarSettings::validate(settings));

    settings.sensor.pcap_rate = 0.0F;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);
}

TEST(LidarSettingsValidate, RejectsAScanThatCannotBeBinned)
{
    auto settings = validSettings();
    settings.scan.min_height = settings.scan.max_height;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings = validSettings();
    settings.scan.angle_min = settings.scan.angle_max;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings = validSettings();
    settings.scan.angle_increment = 0.0;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    // One bin wider than the whole sweep would produce an empty scan.
    settings = validSettings();
    settings.scan.angle_increment = 100.0;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings = validSettings();
    settings.scan.range_min = settings.scan.range_max;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    settings = validSettings();
    settings.scan.scan_time = 0.0;
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);
}

TEST(LidarSettingsValidate, AcceptsSelfFilterBoxes)
{
    auto settings = validSettings();
    settings.scan.self_filter_boxes = {validBox("lidar_support"), validBox("rear_mast")};
    EXPECT_NO_THROW(LidarSettings::validate(settings));
}

TEST(LidarSettingsValidate, RejectsAMalformedSelfFilterBox)
{
    auto settings = validSettings();
    settings.scan.self_filter_boxes = {validBox("")};
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    // An unset bound arrives from the node as NaN.
    settings = validSettings();
    auto unset = validBox("lidar_support");
    unset.max_z = std::numeric_limits<double>::quiet_NaN();
    settings.scan.self_filter_boxes = {unset};
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);

    for (const auto axis : {'x', 'y', 'z'}) {
        auto inverted = validBox("lidar_support");
        double & low = axis == 'x' ? inverted.min_x : axis == 'y' ? inverted.min_y : inverted.min_z;
        double & high = axis == 'x' ? inverted.max_x : axis == 'y' ? inverted.max_y : inverted.max_z;
        low = high;
        settings = validSettings();
        settings.scan.self_filter_boxes = {inverted};
        EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument) << axis;
    }
}

TEST(LidarSettingsValidate, RejectsADuplicateSelfFilterBoxName)
{
    auto settings = validSettings();
    settings.scan.self_filter_boxes = {validBox("lidar_support"), validBox("lidar_support")};
    EXPECT_THROW(LidarSettings::validate(settings), std::invalid_argument);
}
