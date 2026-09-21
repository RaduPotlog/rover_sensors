#!/usr/bin/env bash
#
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
#
# Fails if a file in the given layer includes a ROS or rs_driver header, or an in-project header
# from a layer it must not depend on (see .claude/rules/clean_architecture.md):
#   domain       -> domain/ only
#   application  -> domain/ and application/ only
# The _core library already refuses to compile such an include (it links neither rclcpp nor
# rs_driver); this makes the rule explicit and catches header-only leaks the build would not.
# Adapted from rover_ros/rover_hardware_interface/scripts/check_domain_purity.sh. Invoked from
# CMakeLists.txt as a plain CTest add_test, once per layer.
set -euo pipefail

if [[ $# -ne 2 || ( "$2" != "domain" && "$2" != "application" ) ]]; then
    echo "usage: $0 <package_source_dir> <domain|application>" >&2
    exit 2
fi

PKG_DIR="$1"
LAYER="$2"
PKG="rover_rs16_lidar"
LAYER_DIRS=(
    "${PKG_DIR}/include/${PKG}/${LAYER}"
    "${PKG_DIR}/src/${LAYER}"
)

if [[ "${LAYER}" == "domain" ]]; then
    ALLOWED_QUOTED_PATTERN="^${PKG}/domain/.+\.hpp$"
else
    ALLOWED_QUOTED_PATTERN="^${PKG}/(domain|application)/.+\.hpp$"
fi

# Angle-bracket includes naming any of these are forbidden in both layers.
FORBIDDEN_KEYWORDS=(
    rclcpp
    rclcpp_lifecycle
    rcl_interfaces
    lifecycle_msgs
    diagnostic_updater
    diagnostic_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    # The vendored RoboSense driver and its PCAP input are infrastructure.
    rs_driver
    pcap
)

existing_dirs=()
for dir in "${LAYER_DIRS[@]}"; do
    if [[ -d "${dir}" ]]; then
        existing_dirs+=("${dir}")
    fi
done

if [[ ${#existing_dirs[@]} -eq 0 ]]; then
    echo "check_layer_purity: no ${LAYER} directories found under ${LAYER_DIRS[*]}" >&2
    exit 1
fi

mapfile -t files < <(find "${existing_dirs[@]}" -type f \( -name '*.hpp' -o -name '*.cpp' \) | sort)

status=0

for file in "${files[@]}"; do
    while IFS= read -r include_line; do
        quoted=$(sed -n 's/^[[:space:]]*#include[[:space:]]*"\(.*\)".*$/\1/p' <<< "${include_line}")
        angled=$(sed -n 's/^[[:space:]]*#include[[:space:]]*<\(.*\)>.*$/\1/p' <<< "${include_line}")

        if [[ -n "${quoted}" ]]; then
            if [[ ! "${quoted}" =~ ${ALLOWED_QUOTED_PATTERN} ]]; then
                echo "${LAYER^^} PURITY VIOLATION: ${file} includes '${quoted}'" >&2
                status=1
            fi
        elif [[ -n "${angled}" ]]; then
            for kw in "${FORBIDDEN_KEYWORDS[@]}"; do
                if [[ "${angled}" == *"${kw}"* ]]; then
                    echo "${LAYER^^} PURITY VIOLATION: ${file} includes forbidden header <${angled}>" >&2
                    status=1
                fi
            done
        fi
    done < <(grep -E '^[[:space:]]*#include' "${file}" || true)
done

if [[ ${status} -eq 0 ]]; then
    echo "check_layer_purity: ${LAYER} OK (${#files[@]} files checked)"
fi

exit "${status}"
