cmake_minimum_required(VERSION 3.10)
project(rover_sensors_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Standard ament style/lint gate (lint_cmake, xmllint, copyright, flake8, pep257, ...), as in
  # rover_gps and rover_rs16_lidar. uncrustify is excluded for the same reason as there.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/flake8.ini)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  # E2E: the payload contract (topics, types, QoS, no TF, no foreign subscriptions),
  # checked against the drivers started from the bringup launch file (test/e2e/).
  add_launch_test(test/e2e/test_sensors_contract_launch.py TIMEOUT 90)
endif()

ament_package()
