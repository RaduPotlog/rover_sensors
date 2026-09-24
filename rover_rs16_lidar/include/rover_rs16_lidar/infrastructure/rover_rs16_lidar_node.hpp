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

#ifndef ROVER_RS16_LIDAR_INFRASTRUCTURE_ROVER_RS16_LIDAR_NODE_HPP_
#define ROVER_RS16_LIDAR_INFRASTRUCTURE_ROVER_RS16_LIDAR_NODE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rover_rs16_lidar/application/monitor_lidar_use_case.hpp"
#include "rover_rs16_lidar/application/stream_lidar_use_case.hpp"
#include "rover_rs16_lidar/domain/lidar_health_evaluator.hpp"
#include "rover_rs16_lidar/domain/lidar_settings.hpp"
#include "rover_rs16_lidar/domain/ports/lidar_source_port.hpp"
#include "rover_rs16_lidar/infrastructure/ros2_laser_scan_publisher.hpp"
#include "rover_rs16_lidar/infrastructure/ros2_point_cloud_publisher.hpp"

namespace rover_rs16_lidar
{

/**
 * @brief Composition root: drives the RS16 and publishes its cloud, scan and health.
 * @details Lifecycle-managed because it owns the MSOP/DIFOP UDP sockets, like
 *          rover_gps_driver_node: the sensor is opened on activate and closed on deactivate, so
 *          the ports can be freed for debugging without killing the process. rs_driver only
 *          closes its sockets when the driver object is destroyed (stop() leaves them bound),
 *          so a fresh source is built on every activate and dropped on every deactivate.
 *
 *          Health diagnostics run from configure to cleanup, so while the node is inactive the
 *          lidar reports STALE and then a cloud timeout, which is the truth.
 */
class RoverRs16LidarNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /// Builds the sensor to read from on each activate.
    using SourceFactory =
        std::function<std::shared_ptr<domain::LidarSourcePort>(const domain::SensorSettings &)>;

    /**
     * @param source_factory `nullptr` uses the real RS16 over rs_driver. Tests inject a stub
     *        here so the whole pipeline runs with no hardware on the network.
     * @throws std::invalid_argument when the parameter overrides are inconsistent.
     */
    RoverRs16LidarNode(
        const std::string & node_name, const std::string & ns = "/",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
        SourceFactory source_factory = nullptr);

    ~RoverRs16LidarNode() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    const domain::LidarSettings & settings() const { return settings_; }

private:
    void declareParameters();

    /** @brief Stops the source and destroys it, which is what releases the UDP ports. */
    void closeSource();

    /** @brief Drops everything on_configure built. */
    void releasePipeline();

    void onFrame(const domain::PointCloudFrame & cloud);

    void onSourceError(const std::string & message);

    void tickCallback();

    double nowSeconds() const;

    domain::LidarSettings settings_;
    domain::LidarHealthThresholds health_thresholds_;
    double publish_frequency_{1.0};
    SourceFactory source_factory_;

    // Declared before monitor_lidar_ so it outlives it: the health publisher that use case
    // owns unregisters its diagnostic task from this Updater when destroyed.
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    std::shared_ptr<application::MonitorLidarUseCase> monitor_lidar_;
    std::shared_ptr<infrastructure::Ros2PointCloudPublisher> cloud_publisher_;
    std::shared_ptr<infrastructure::Ros2LaserScanPublisher> scan_publisher_;
    std::unique_ptr<application::StreamLidarUseCase> stream_lidar_;
    std::shared_ptr<domain::LidarSourcePort> source_;

    rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace rover_rs16_lidar

#endif  // ROVER_RS16_LIDAR_INFRASTRUCTURE_ROVER_RS16_LIDAR_NODE_HPP_
