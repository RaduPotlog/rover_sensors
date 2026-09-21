cmake_minimum_required(VERSION 3.10)
project(rover_rs16_lidar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    diagnostic_msgs
    diagnostic_updater
    rcl_interfaces
    sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_package(Threads REQUIRED)
find_library(PCAP_LIBRARY NAMES pcap REQUIRED)

# Vendored RoboSense rs_driver 1.5.20 (header-only) - see 3rdparty/rs_driver/README.rover.md.
# Its own CMake project is deliberately not used: it hardcodes CMAKE_INSTALL_PREFIX, forces
# -std=c++14, and ships a generated config with absolute paths. SYSTEM keeps the vendor's
# warnings out of our -Wall -Wextra -Wpedantic build.
add_library(rs_driver_vendor INTERFACE)
target_include_directories(rs_driver_vendor SYSTEM INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rs_driver>
)
# Must stay in step with the feature set rover_rslidar_sdk was built with.
target_compile_definitions(rs_driver_vendor INTERFACE
    POINT_TYPE_XYZI        # x/y/z/intensity only - no ring, no per-point timestamp
    ENABLE_MODIFY_RECVBUF
    ENABLE_DIFOP_PARSE
)
target_link_libraries(rs_driver_vendor INTERFACE Threads::Threads ${PCAP_LIBRARY})

# Domain + application: plain C++, no ROS and no rs_driver dependencies.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/lidar_health_evaluator.cpp
            src/domain/lidar_settings.cpp
            src/domain/scan_projector.cpp
            src/application/monitor_lidar_use_case.cpp
            src/application/stream_lidar_use_case.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Infrastructure: the rs_driver adapter and the ROS 2 adapters.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/rs_driver_lidar_source.cpp
            src/infrastructure/ros2_point_cloud_publisher.cpp
            src/infrastructure/ros2_laser_scan_publisher.cpp
            src/infrastructure/ros2_lidar_health_publisher.cpp
            src/infrastructure/rover_rs16_lidar_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME}_core rs_driver_vendor)
target_link_libraries(${PROJECT_NAME}_ros
    ${rclcpp_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${rcl_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/rover_rs16_lidar.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Standard ament style/lint gate (cpplint, cppcheck, lint_cmake, xmllint, copyright, ...), as
  # in rover_ros/rover_hardware_interface. uncrustify is excluded for the same reason as there:
  # this repo uses 4-space indentation, not ament_cmake_uncrustify's default style.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  # The vendored rs_driver keeps upstream's style; lint only our own code.
  file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/flake8.ini)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Domain and application must stay ROS-free (see .claude/rules/clean_architecture.md). Plain
  # CTest add_test: pass/fail via exit code, no JUnit-XML contract to satisfy.
  add_test(
    NAME domain_purity_check
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_layer_purity.sh
            ${CMAKE_CURRENT_SOURCE_DIR} domain
  )
  add_test(
    NAME application_purity_check
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_layer_purity.sh
            ${CMAKE_CURRENT_SOURCE_DIR} application
  )

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lidar_health_evaluator test/unit/test_lidar_health_evaluator.cpp)
  target_link_libraries(test_lidar_health_evaluator ${PROJECT_NAME}_core)

  ament_add_gtest(test_lidar_settings test/unit/test_lidar_settings.cpp)
  target_link_libraries(test_lidar_settings ${PROJECT_NAME}_core)

  ament_add_gtest(test_scan_projector test/unit/test_scan_projector.cpp)
  target_link_libraries(test_scan_projector ${PROJECT_NAME}_core)

  ament_add_gtest(test_stream_lidar_use_case test/unit/test_stream_lidar_use_case.cpp)
  target_link_libraries(test_stream_lidar_use_case ${PROJECT_NAME}_core)

  ament_add_gtest(test_rover_rs16_lidar_node test/integration/test_rover_rs16_lidar_node.cpp
                  TIMEOUT 60)
  target_link_libraries(test_rover_rs16_lidar_node ${PROJECT_NAME}_ros ${lifecycle_msgs_TARGETS})
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
