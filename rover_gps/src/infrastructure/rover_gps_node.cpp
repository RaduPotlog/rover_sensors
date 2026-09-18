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

#include "rover_gps/infrastructure/rover_gps_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "rover_gps/infrastructure/ros2_gps_health_publisher.hpp"

namespace rover_gps
{

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describePositive(
    const std::string & description, double max_value)
{
    auto descriptor = describe(description);
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 1.0e-6;
    descriptor.floating_point_range[0].to_value = max_value;
    return descriptor;
}

}  // namespace

RoverGpsNode::RoverGpsNode(
    const std::string & node_name,
    const std::string & ns,
    const rclcpp::NodeOptions & options)
: Node(node_name, ns, options)
, diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
    const domain::GpsHealthThresholds health_defaults;
    health_thresholds_.expected_rate_hz = declare_parameter(
        "expected_rate_hz", health_defaults.expected_rate_hz,
        describePositive("GGA sentence rate the receiver is configured for [Hz].", 100.0));
    health_thresholds_.min_rate_ratio = declare_parameter(
        "min_rate_ratio", health_defaults.min_rate_ratio,
        describePositive("Warn below expected_rate_hz * min_rate_ratio.", 1.0));
    health_thresholds_.fix_timeout_s = declare_parameter(
        "fix_timeout_s", health_defaults.fix_timeout_s,
        describePositive("Report an error when no fix arrives for this long [s].", 600.0));
    health_thresholds_.warn_horizontal_std_m = declare_parameter(
        "warn_horizontal_std_m", health_defaults.warn_horizontal_std_m,
        describePositive("Warn above this horizontal 1-sigma error [m].", 1.0e4));
    health_thresholds_.error_horizontal_std_m = declare_parameter(
        "error_horizontal_std_m", health_defaults.error_horizontal_std_m,
        describePositive("Report an error above this horizontal 1-sigma error [m].", 1.0e4));

    // Reject inconsistent combinations (e.g. warn > error) at startup, not at the first fix.
    domain::GpsHealthEvaluator::validate(health_thresholds_);
}

void RoverGpsNode::init()
{
    diagnostic_updater_->setHardwareID("RoverGps");

    monitor_gps_ = std::make_unique<application::MonitorGpsUseCase>(
        std::make_shared<infrastructure::Ros2GpsHealthPublisher>(diagnostic_updater_),
        health_thresholds_);

    fix_subscriber_ = create_subscription<infrastructure::NavSatFixMsg>(
        "gps/fix", rclcpp::SensorDataQoS(), std::bind(&RoverGpsNode::fixCallback, this, _1));

    tick_timer_ = create_wall_timer(1s, std::bind(&RoverGpsNode::tickCallback, this));

    // Publish the STALE status immediately instead of after the first timer period.
    tickCallback();
}

void RoverGpsNode::fixCallback(const infrastructure::NavSatFixMsg::SharedPtr msg)
{
    const domain::GnssFix fix = infrastructure::toGnssFix(*msg, nowSeconds());
    monitor_gps_->onFix(fix);
}

void RoverGpsNode::tickCallback()
{
    monitor_gps_->onTick(nowSeconds());
}

double RoverGpsNode::nowSeconds() const
{
    return get_clock()->now().seconds();
}

}  // namespace rover_gps
