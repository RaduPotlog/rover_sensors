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

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rover_gps/infrastructure/rover_gps_node.hpp"

using namespace std::chrono_literals;
using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;
using NavSatFixMsg = sensor_msgs::msg::NavSatFix;

namespace
{

constexpr char kNamespace[] = "/rover_gps_test";

class RoverGpsNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
    static void TearDownTestSuite() {rclcpp::shutdown();}

    void startNode(const std::vector<rclcpp::Parameter> & overrides)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(overrides);
        gps_node_ = std::make_shared<rover_gps::RoverGpsNode>("rover_gps_node", kNamespace, options);
        gps_node_->init();

        tester_ = std::make_shared<rclcpp::Node>("tester", kNamespace);
        fix_pub_ = tester_->create_publisher<NavSatFixMsg>("gps/fix", 10);
        // diagnostic_updater publishes on the absolute /diagnostics topic.
        diagnostics_sub_ = tester_->create_subscription<DiagnosticArrayMsg>(
            "/diagnostics", 10,
            [this](DiagnosticArrayMsg::SharedPtr msg) {
                for (const auto & status : msg->status) {
                    latest_statuses_[status.name] = status;
                }
            });

        executor_.add_node(gps_node_);
        executor_.add_node(tester_);
    }

    void TearDown() override
    {
        // Tests that only construct a node never call startNode().
        if (tester_) {
            executor_.remove_node(tester_);
        }
        if (gps_node_) {
            executor_.remove_node(gps_node_);
        }
    }

    /** Spins until `done` returns true or `timeout` elapses; returns `done()`. */
    bool spinUntil(
        const std::function<bool()> & done, std::chrono::milliseconds timeout,
        const std::function<void()> & each_iteration = {})
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            if (each_iteration) {
                each_iteration();
            }
            executor_.spin_some(20ms);
        }
        return done();
    }

    bool waitForDiscovery()
    {
        return spinUntil(
            [this] {
                return fix_pub_->get_subscription_count() > 0 &&
                       diagnostics_sub_->get_publisher_count() > 0;
            }, 5s);
    }

    std::optional<DiagnosticStatusMsg> status(const std::string & task) const
    {
        const auto it = latest_statuses_.find("rover_gps_node: " + task);
        if (it == latest_statuses_.end()) {
            return std::nullopt;
        }
        return it->second;
    }

    /** Publishes a fix with a 1 m horizontal error. */
    void publishFix()
    {
        NavSatFixMsg fix;
        fix.latitude = 45.0;
        fix.longitude = 25.0;
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        fix.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_APPROXIMATED;
        fix.position_covariance[0] = 1.0;
        fix.position_covariance[4] = 1.0;
        fix_pub_->publish(fix);
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rover_gps::RoverGpsNode> gps_node_;
    rclcpp::Node::SharedPtr tester_;
    rclcpp::Publisher<NavSatFixMsg>::SharedPtr fix_pub_;
    rclcpp::Subscription<DiagnosticArrayMsg>::SharedPtr diagnostics_sub_;
    std::map<std::string, DiagnosticStatusMsg> latest_statuses_;
};

}  // namespace

TEST_F(RoverGpsNodeTest, ReportsStaleBeforeData)
{
    startNode({});
    ASSERT_TRUE(waitForDiscovery());

    ASSERT_TRUE(spinUntil([this] {return status("GPS fix").has_value();}, 5s));
    EXPECT_EQ(status("GPS fix")->level, DiagnosticStatusMsg::STALE);
}

TEST_F(RoverGpsNodeTest, ReportsFixDiagnostics)
{
    startNode({});
    ASSERT_TRUE(waitForDiscovery());

    ASSERT_TRUE(spinUntil(
        [this] {
            const auto fix_status = status("GPS fix");
            return fix_status && fix_status->level == DiagnosticStatusMsg::OK;
        }, 10s, [this] {publishFix();}));
}

TEST_F(RoverGpsNodeTest, RejectsInconsistentThresholds)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("warn_horizontal_std_m", 50.0),
        rclcpp::Parameter("error_horizontal_std_m", 10.0),
    });
    EXPECT_THROW(
        rover_gps::RoverGpsNode("rover_gps_node", kNamespace, options), std::invalid_argument);
}
