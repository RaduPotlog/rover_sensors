#!/usr/bin/env python3

# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def env_flag(value):
    """True for true/1/yes/on (any case): the values usually come straight from balena."""
    return PythonExpression(["'", value, "'.strip().lower() in ('true', '1', 'yes', 'on')"])


def generate_launch_description():

    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "FATAL"],
        description="Logging level",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROVER_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    # ROVER_USE_GPS is one switch for GPS on the whole rover: this driver here, and the GPS
    # fusion in the platform's localization.
    use_gps = LaunchConfiguration("use_gps")
    declare_use_gps_arg = DeclareLaunchArgument(
        "use_gps",
        default_value=EnvironmentVariable("ROVER_USE_GPS", default_value="false"),
        description="Start the RUTX11 NMEA GNSS driver (true/false).",
    )

    use_lidar = LaunchConfiguration("use_lidar")
    declare_use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value=EnvironmentVariable("ROVER_USE_LIDAR", default_value="false"),
        description="Start the RoboSense RS16 lidar driver (true/false).",
    )

    common_launch_arguments = {
        "log_level": log_level,
        "namespace": namespace,
        "common_dir_path": common_dir_path,
    }.items()

    rover_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rover_gps"), "launch", "rover_gps.launch.py"]
            )
        ),
        condition=IfCondition(env_flag(use_gps)),
        launch_arguments=common_launch_arguments,
    )

    rover_rs16_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rover_rs16_lidar"), "launch", "rover_rs16_lidar.launch.py"]
            )
        ),
        condition=IfCondition(env_flag(use_lidar)),
        launch_arguments=common_launch_arguments,
    )

    return LaunchDescription(
        [
            declare_common_dir_path_arg,
            declare_log_level_arg,
            declare_namespace_arg,
            declare_use_gps_arg,
            declare_use_lidar_arg,
            rover_gps_launch,
            rover_rs16_lidar_launch,
        ]
    )
