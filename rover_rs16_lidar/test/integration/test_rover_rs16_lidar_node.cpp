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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rover_rs16_lidar/infrastructure/rover_rs16_lidar_node.hpp"

namespace
{

using namespace std::chrono_literals;

using rover_rs16_lidar::RoverRs16LidarNode;
using rover_rs16_lidar::domain::LidarPoint;
using rover_rs16_lidar::domain::LidarSourcePort;
using rover_rs16_lidar::domain::PointCloudFrame;
using rover_rs16_lidar::domain::SensorSettings;
using lifecycle_msgs::msg::State;

/**
 * @brief A sensor that is driven by the test rather than by UDP packets.
 * @details This is the whole point of LidarSourcePort: the pipeline below the driver runs
 *          identically whether the frames come from an RS16 or from here.
 */
class StubLidarSource : public LidarSourcePort
{
public:
    void setFrameCallback(FrameCallback callback) override { on_frame_ = std::move(callback); }
    void setErrorCallback(ErrorCallback callback) override { on_error_ = std::move(callback); }
    void start() override
    {
        if (fail_on_start) {
            throw std::runtime_error("UDP port 6699 is already in use");
        }
        started = true;
    }
    void stop() override { stopped = true; }

    void emit(const PointCloudFrame & cloud)
    {
        ASSERT_TRUE(on_frame_) << "the node must register a frame callback before start()";
        on_frame_(cloud);
    }

    void emitError(const std::string & message)
    {
        if (on_error_) {
            on_error_(message);
        }
    }

    bool fail_on_start{false};
    bool started{false};
    bool stopped{false};

private:
    FrameCallback on_frame_;
    ErrorCallback on_error_;
};

PointCloudFrame sampleCloud()
{
    PointCloudFrame cloud;
    cloud.points = {
        LidarPoint{3.0F, 0.0F, 0.0F, 11.0F},
        LidarPoint{0.0F, 4.0F, 0.1F, 12.0F},
        LidarPoint{-5.0F, 0.0F, -0.1F, 13.0F},
    };
    cloud.width = 3;
    cloud.height = 1;
    cloud.timestamp_s = 1000.25;
    cloud.is_dense = false;
    return cloud;
}

rclcpp::NodeOptions optionsWith(std::vector<rclcpp::Parameter> overrides)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides(std::move(overrides));
    return options;
}

/// True when nobody holds `port`. Deliberately without SO_REUSEADDR, so it fails while
/// rs_driver (which sets it) still has the port bound.
bool canBind(std::uint16_t port)
{
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);

    const bool bound = ::bind(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0;
    ::close(fd);
    return bound;
}

/// Two distinct UDP ports the kernel just reported free.
std::pair<std::uint16_t, std::uint16_t> freeUdpPorts()
{
    int fds[2];
    std::uint16_t ports[2] = {0, 0};
    for (int i = 0; i < 2; ++i) {
        fds[i] = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = 0;
        ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
        ::bind(fds[i], reinterpret_cast<const sockaddr *>(&address), sizeof(address));
        socklen_t length = sizeof(address);
        ::getsockname(fds[i], reinterpret_cast<sockaddr *>(&address), &length);
        ports[i] = ntohs(address.sin_port);
    }
    // Both held open until now, so the two ports differ.
    ::close(fds[0]);
    ::close(fds[1]);
    return {ports[0], ports[1]};
}

class RoverRs16LidarNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        observer_ = std::make_shared<rclcpp::Node>("rover_rs16_lidar_test_observer");
        executor_->add_node(observer_);
    }

    void TearDown() override
    {
        executor_.reset();
        node_.reset();
        observer_.reset();
        rclcpp::shutdown();
    }

    /// Builds the node, unconfigured, with every activate handed a fresh stub source.
    void makeNode(std::vector<rclcpp::Parameter> overrides = {})
    {
        node_ = std::make_shared<RoverRs16LidarNode>(
            "rover_rs16_lidar_node", "/", optionsWith(std::move(overrides)),
            [this](const SensorSettings &) {
                ++factory_calls_;
                source_ = std::make_shared<StubLidarSource>();
                source_->fail_on_start = fail_next_start_;
                return source_;
            });
        executor_->add_node(node_->get_node_base_interface());
    }

    /// Builds the node and drives it to active, with the stub source wired in and started.
    void buildNode(std::vector<rclcpp::Parameter> overrides = {})
    {
        makeNode(std::move(overrides));
        ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
        ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
        ASSERT_TRUE(source_);
    }

    /// Spins both nodes so the stub's frames make it across the middleware.
    void pump(std::chrono::milliseconds budget = 500ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + budget;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_->spin_some();
            std::this_thread::sleep_for(10ms);
        }
    }

    /// Spins until `predicate` holds or the budget runs out - never a bare sleep.
    template <typename Predicate>
    bool pumpUntil(Predicate predicate, std::chrono::milliseconds budget = 5s)
    {
        const auto deadline = std::chrono::steady_clock::now() + budget;
        while (std::chrono::steady_clock::now() < deadline) {
            if (predicate()) {
                return true;
            }
            executor_->spin_some();
            std::this_thread::sleep_for(10ms);
        }
        return predicate();
    }

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::shared_ptr<RoverRs16LidarNode> node_;
    std::shared_ptr<StubLidarSource> source_;
    int factory_calls_{0};
    bool fail_next_start_{false};
    rclcpp::Node::SharedPtr observer_;
};

}  // namespace

TEST_F(RoverRs16LidarNodeTest, DeclaresTheShippedDefaults)
{
    buildNode();

    EXPECT_EQ(node_->get_parameter("point_cloud_topic").as_string(), "rslidar_points");
    EXPECT_EQ(node_->get_parameter("scan_topic").as_string(), "scan");
    EXPECT_TRUE(node_->get_parameter("publish_scan").as_bool());
    EXPECT_EQ(node_->get_parameter("msop_port").as_int(), 6699);
    EXPECT_EQ(node_->get_parameter("difop_port").as_int(), 7788);
    EXPECT_DOUBLE_EQ(node_->get_parameter("expected_rate_hz").as_double(), 10.0);
    EXPECT_DOUBLE_EQ(node_->get_parameter("scan.range_max").as_double(), 20.0);
    EXPECT_TRUE(source_->started);
}

TEST_F(RoverRs16LidarNodeTest, RejectsInconsistentOverridesAtConstruction)
{
    // min_distance above max_distance can never produce a point.
    EXPECT_THROW(
        std::make_shared<RoverRs16LidarNode>(
            "rover_rs16_lidar_node", "/",
            optionsWith({rclcpp::Parameter("min_distance", 50.0),
                         rclcpp::Parameter("max_distance", 10.0)})),
        std::invalid_argument);

    // pcap replay with no file to replay.
    EXPECT_THROW(
        std::make_shared<RoverRs16LidarNode>(
            "rover_rs16_lidar_node", "/",
            optionsWith({rclcpp::Parameter("input_type", "pcap")})),
        std::invalid_argument);

    EXPECT_THROW(
        std::make_shared<RoverRs16LidarNode>(
            "rover_rs16_lidar_node", "/",
            optionsWith({rclcpp::Parameter("input_type", "rosbag")})),
        std::invalid_argument);
}

TEST_F(RoverRs16LidarNodeTest, PublishesThePointCloudAFrameProduces)
{
    buildNode({rclcpp::Parameter("frame_id", "rover/lidar_link")});

    sensor_msgs::msg::PointCloud2::SharedPtr received;
    auto subscription = observer_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", rclcpp::SensorDataQoS(),
        [&received](sensor_msgs::msg::PointCloud2::SharedPtr msg) { received = msg; });

    // The publisher skips clouds nobody subscribes to, so wait until it sees this one.
    ASSERT_TRUE(pumpUntil([this] { return node_->count_subscribers("/rslidar_points") > 0; }));
    source_->emit(sampleCloud());
    ASSERT_TRUE(pumpUntil([&received] { return received != nullptr; }));

    EXPECT_EQ(received->header.frame_id, "rover/lidar_link");
    // Stamp is split the way rslidar_sdk split it, so consumers see the same instant.
    EXPECT_EQ(received->header.stamp.sec, 1000);
    EXPECT_EQ(received->header.stamp.nanosec, 250000000U);

    // Layout must stay byte-identical to what rslidar_sdk produced with POINT_TYPE_XYZI:
    // four FLOAT32 fields, 16-byte stride, and width/height swapped for pcl.
    ASSERT_EQ(received->fields.size(), 4U);
    EXPECT_EQ(received->fields[0].name, "x");
    EXPECT_EQ(received->fields[3].name, "intensity");
    for (const auto & field : received->fields) {
        EXPECT_EQ(field.datatype, sensor_msgs::msg::PointField::FLOAT32);
        EXPECT_EQ(field.count, 1U);
    }
    EXPECT_EQ(received->point_step, 16U);
    EXPECT_EQ(received->width, 1U);   // the cloud's height
    EXPECT_EQ(received->height, 3U);  // the cloud's width
    EXPECT_EQ(received->row_step, 16U);
    EXPECT_FALSE(received->is_dense);
    ASSERT_EQ(received->data.size(), 3U * 16U);

    float x = 0.0F;
    std::memcpy(&x, received->data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(x, 3.0F);
}

TEST_F(RoverRs16LidarNodeTest, PublishesTheFlattenedScan)
{
    buildNode();

    sensor_msgs::msg::LaserScan::SharedPtr received;
    auto subscription = observer_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [&received](sensor_msgs::msg::LaserScan::SharedPtr msg) { received = msg; });

    pump(300ms);
    source_->emit(sampleCloud());
    ASSERT_TRUE(pumpUntil([&received] { return received != nullptr; }));

    // Full sweep at half a degree.
    EXPECT_EQ(received->ranges.size(), 720U);
    EXPECT_FLOAT_EQ(received->range_max, 20.0F);
    // All three sample points sit in the height band and inside 20 m, so three beams see them.
    const auto hits = std::count_if(
        received->ranges.begin(), received->ranges.end(),
        [](float range) { return std::isfinite(range); });
    EXPECT_EQ(hits, 3);
}

TEST_F(RoverRs16LidarNodeTest, PublishesNoScanWhenPublishScanIsOff)
{
    buildNode({rclcpp::Parameter("publish_scan", false)});

    bool got_scan = false;
    auto subscription = observer_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [&got_scan](sensor_msgs::msg::LaserScan::SharedPtr) { got_scan = true; });

    pump(300ms);
    source_->emit(sampleCloud());
    pump(500ms);

    EXPECT_FALSE(got_scan);
}

TEST_F(RoverRs16LidarNodeTest, ReportsStaleBeforeAnyFrameAndOkAfterOne)
{
    // The Updater publishes on its own timer; shorten both so the test is not slow.
    buildNode({rclcpp::Parameter("publish_frequency", 20.0),
               rclcpp::Parameter("diagnostic_updater.period", 0.1)});

    std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses;
    auto subscription = observer_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10,
        [&statuses](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
            for (const auto & status : msg->status) {
                statuses.push_back(status);
            }
        });

    // The name downstream consumers match on: rover_diag_manager strips the node prefix and
    // rover_navigation's IsLidarHealthy / rover_mission_manager match the whole string.
    const std::string expected_name = "rover_rs16_lidar_node: Lidar status";

    pump(800ms);
    ASSERT_FALSE(statuses.empty()) << "no diagnostics published";
    EXPECT_EQ(statuses.back().name, expected_name);
    EXPECT_EQ(statuses.back().hardware_id, "RoverLidar");
    EXPECT_EQ(statuses.back().level, diagnostic_msgs::msg::DiagnosticStatus::STALE);

    statuses.clear();
    source_->emit(sampleCloud());
    pump(800ms);

    ASSERT_FALSE(statuses.empty());
    EXPECT_EQ(statuses.back().name, expected_name);
    // Three points is well under min_points_warn, so a warning rather than OK is expected.
    EXPECT_EQ(statuses.back().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST_F(RoverRs16LidarNodeTest, ReportsErrorWhenNoFrameArrivesWithinStartupGrace)
{
    // A lidar missing at power-up must not stay STALE forever.
    buildNode({rclcpp::Parameter("publish_frequency", 20.0),
               rclcpp::Parameter("diagnostic_updater.period", 0.1),
               rclcpp::Parameter("startup_grace_s", 0.3)});

    std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses;
    auto subscription = observer_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10,
        [&statuses](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
            for (const auto & status : msg->status) {
                statuses.push_back(status);
            }
        });

    pump(800ms);

    ASSERT_FALSE(statuses.empty()) << "no diagnostics published";
    EXPECT_EQ(statuses.back().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    EXPECT_EQ(statuses.back().message, "No lidar data since startup.");
}

TEST_F(RoverRs16LidarNodeTest, StopsTheSourceWhenDestroyed)
{
    buildNode();
    auto source = source_;

    executor_->remove_node(node_->get_node_base_interface());
    node_.reset();

    EXPECT_TRUE(source->stopped);
}

TEST_F(RoverRs16LidarNodeTest, SurvivesADriverError)
{
    buildNode();

    // A driver-level error is logged, never thrown across rs_driver's thread.
    EXPECT_NO_THROW(source_->emitError("ERRCODE_MSOPTIMEOUT"));

    sensor_msgs::msg::PointCloud2::SharedPtr received;
    auto subscription = observer_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", rclcpp::SensorDataQoS(),
        [&received](sensor_msgs::msg::PointCloud2::SharedPtr msg) { received = msg; });

    // The publisher skips clouds nobody subscribes to, so wait until it sees this one.
    ASSERT_TRUE(pumpUntil([this] { return node_->count_subscribers("/rslidar_points") > 0; }));
    source_->emit(sampleCloud());
    EXPECT_TRUE(pumpUntil([&received] { return received != nullptr; }));
}

TEST_F(RoverRs16LidarNodeTest, DeactivateStopsAndDropsTheSource)
{
    buildNode();
    auto first = source_;
    source_.reset();

    ASSERT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(first->stopped);
    // rs_driver only closes its sockets when destroyed, so the node must let go of it.
    EXPECT_EQ(first.use_count(), 1) << "the node still holds the source after deactivate";

    ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(factory_calls_, 2);
    ASSERT_TRUE(source_);
    EXPECT_NE(source_, first);
    EXPECT_TRUE(source_->started);

    sensor_msgs::msg::PointCloud2::SharedPtr received;
    auto subscription = observer_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", rclcpp::SensorDataQoS(),
        [&received](sensor_msgs::msg::PointCloud2::SharedPtr msg) { received = msg; });

    // The publisher skips clouds nobody subscribes to, so wait until it sees this one.
    ASSERT_TRUE(pumpUntil([this] { return node_->count_subscribers("/rslidar_points") > 0; }));
    source_->emit(sampleCloud());
    EXPECT_TRUE(pumpUntil([&received] { return received != nullptr; }))
        << "no cloud after a deactivate/activate cycle";
}

TEST_F(RoverRs16LidarNodeTest, ActivateFailsWhenTheSourceCannotStart)
{
    fail_next_start_ = true;
    makeNode();
    ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);

    // A port conflict leaves the node inactive instead of crashing the process.
    EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(source_);
    EXPECT_EQ(source_.use_count(), 1) << "a source that failed to start must not be kept";

    // Once the port is free again, a retry succeeds.
    fail_next_start_ = false;
    EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
    EXPECT_TRUE(source_->started);
}

TEST_F(RoverRs16LidarNodeTest, CleanupReturnsToUnconfigured)
{
    buildNode();

    ASSERT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(RoverRs16LidarNodeTest, DeactivateReleasesTheUdpPorts)
{
    // The real rs_driver source, on ports no one else is using. No lidar is needed: the
    // sockets are bound whether or not packets ever arrive.
    const auto [msop_port, difop_port] = freeUdpPorts();
    node_ = std::make_shared<RoverRs16LidarNode>(
        "rover_rs16_lidar_node", "/",
        optionsWith({rclcpp::Parameter("msop_port", static_cast<int>(msop_port)),
                     rclcpp::Parameter("difop_port", static_cast<int>(difop_port)),
                     rclcpp::Parameter("host_address", "127.0.0.1")}));

    ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
    // Held while active.
    EXPECT_FALSE(canBind(msop_port));
    EXPECT_FALSE(canBind(difop_port));

    ASSERT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(canBind(msop_port)) << "MSOP port not released on deactivate";
    EXPECT_TRUE(canBind(difop_port)) << "DIFOP port not released on deactivate";

    // And bound again on the next activate.
    ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
    EXPECT_FALSE(canBind(msop_port));
}
