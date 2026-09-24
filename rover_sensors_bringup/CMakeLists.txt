cmake_minimum_required(VERSION 3.10)
project(rover_sensors_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  # E2E: the payload contract (topics, types, QoS, no TF, no foreign subscriptions),
  # checked against the drivers started from the bringup launch file (test/e2e/).
  add_launch_test(test/e2e/test_sensors_contract_launch.py TIMEOUT 90)
endif()

ament_package()
