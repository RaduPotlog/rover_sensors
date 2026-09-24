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

#include <exception>
#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rover_gps/infrastructure/rover_gps_driver_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    const auto logger = rclcpp::get_logger("rover_gps_driver");

    std::shared_ptr<rover_gps::RoverGpsDriverNode> node;
    try {
        // Construction can throw on an invalid parameter override.
        node = std::make_shared<rover_gps::RoverGpsDriverNode>("rover_gps_driver");
    } catch (const std::exception & e) {
        RCLCPP_FATAL(logger, "Caught exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    // On Ctrl-C, run the lifecycle shutdown transition while the context is still valid, so
    // on_shutdown() stops the receiver and releases the sockets instead of the destructor doing
    // it on an active node. The callback runs on rclcpp's signal thread: stop the executor and
    // wait for spin() to return first, so the transition never races the executor thread.
    std::promise<void> spin_exited;
    std::shared_future<void> spin_exited_future = spin_exited.get_future().share();
    auto context = node->get_node_base_interface()->get_context();
    const auto pre_shutdown_handle = context->add_pre_shutdown_callback(
        [&executor, &node, spin_exited_future]() {
            executor.cancel();
            spin_exited_future.wait();
            node->shutdown();
        });

    int exit_code = 0;
    try {
        executor.spin();
    } catch (const std::exception & e) {
        RCLCPP_FATAL(logger, "Caught exception: %s", e.what());
        exit_code = 1;
    }
    spin_exited.set_value();

    RCLCPP_INFO(logger, "Shutting down");

    // After Ctrl-C this waits for the callback above to finish; otherwise it runs it.
    rclcpp::shutdown();
    context->remove_pre_shutdown_callback(pre_shutdown_handle);

    return exit_code;
}
