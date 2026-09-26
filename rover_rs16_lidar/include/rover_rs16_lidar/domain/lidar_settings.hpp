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

#ifndef ROVER_RS16_LIDAR_DOMAIN_LIDAR_SETTINGS_HPP_
#define ROVER_RS16_LIDAR_DOMAIN_LIDAR_SETTINGS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rover_rs16_lidar::domain
{

/** @brief Where the MSOP/DIFOP packets come from. */
enum class LidarInputType
{
    /// Live sensor on the network.
    Lidar,
    /// Replay of a capture - how the stack is exercised without hardware.
    Pcap,
};

/** @throws std::invalid_argument on anything but "lidar" or "pcap". */
LidarInputType parseInputType(const std::string & value);

/** @brief How the RS16 itself is driven. Mirrors rs_driver's RSInputParam/RSDecoderParam. */
struct SensorSettings
{
    LidarInputType input_type{LidarInputType::Lidar};

    std::uint16_t msop_port{6699};
    std::uint16_t difop_port{7788};
    /// Interface to bind; 0.0.0.0 accepts the stream on any of them.
    std::string host_address{"0.0.0.0"};
    /// Multicast group, when the lidar is configured to multicast.
    std::string group_address{"0.0.0.0"};

    float min_distance{0.2F};
    float max_distance{100.0F};
    /// True stamps clouds with the lidar's own clock, which is not comparable to the rover's.
    bool use_lidar_clock{false};
    /// False keeps invalid returns as NaN - the scan projector drops them.
    bool dense_points{false};
    bool ts_first_point{true};
    /// False starts publishing before the calibration DIFOP has been seen.
    bool wait_for_difop{true};
    float start_angle{0.0F};
    float end_angle{360.0F};

    std::string pcap_path;
    bool pcap_repeat{false};
    float pcap_rate{1.0F};
};

/**
 * @brief A box, in the lidar's own frame, whose returns are the rover itself.
 * @details The scan slice can take in parts of the rover that sit within the RS16's field of
 *          view. On Rover A1 that is the lidar's support post, about 0.4 m to the lidar's left:
 *          the costmaps marked it as a lethal obstacle on the footprint edge, where Nav 2's
 *          footprint clearing does not reliably reach, so goals near the rover read as
 *          occupied and MPPI saw every trajectory as a collision. Axis-aligned in the sensor
 *          frame because that is the frame the cloud arrives in - no TF is involved, and a box
 *          moves with the lidar. Re-measure it (scripts/measure_self_filter.py) whenever the
 *          lidar or its mount moves.
 */
struct SelfFilterBox
{
    /// Parameter namespace of the box: scan.self_filter.<name>.min_x and so on.
    std::string name;
    double min_x{0.0};
    double max_x{0.0};
    double min_y{0.0};
    double max_y{0.0};
    double min_z{0.0};
    double max_z{0.0};

    /** @brief True when the point is inside the box, faces included. */
    bool contains(double x, double y, double z) const
    {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z &&
               z <= max_z;
    }
};

/** @brief The horizontal slice published as a LaserScan for the Nav 2 costmaps. */
struct ScanSettings
{
    bool enabled{true};
    /// Band around the lidar's own origin, out of the RS16's +/-15 degree fan.
    double min_height{-0.25};
    double max_height{0.25};
    double angle_min{-3.141592653589793};
    double angle_max{3.141592653589793};
    double angle_increment{0.008726646259971648};  // 0.5 degrees
    double scan_time{0.1};                         // 10 Hz, the RS16 default spin rate
    double range_min{0.2};
    double range_max{20.0};
    /// True reports a beam with no return as +inf, false as range_max + 1.
    bool use_inf{true};
    /// Returns inside any of these are the rover itself and are left out of the scan. The
    /// published point cloud is not filtered.
    std::vector<SelfFilterBox> self_filter_boxes;
};

/**
 * @brief Rejects a box that could not filter anything sensible.
 * @throws std::invalid_argument naming the box and the offending bound.
 */
void validateSelfFilterBox(const SelfFilterBox & box);

/** @brief Everything the node is configured with, already validated. */
struct LidarSettings
{
    std::string frame_id{"lidar_link"};
    std::string point_cloud_topic{"rslidar_points"};
    std::string scan_topic{"scan"};
    std::size_t publisher_queue_size{10};

    SensorSettings sensor;
    ScanSettings scan;

    /**
     * @brief Rejects inconsistent configuration at startup rather than at the first cloud.
     * @throws std::invalid_argument naming the offending field.
     */
    static void validate(const LidarSettings & settings);
};

}  // namespace rover_rs16_lidar::domain

#endif  // ROVER_RS16_LIDAR_DOMAIN_LIDAR_SETTINGS_HPP_
