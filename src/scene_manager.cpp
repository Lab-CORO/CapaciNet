#include "rclcpp/rclcpp.hpp"
#include "curobo_msgs/srv/generate_rm.hpp"
#include "curobo_msgs/srv/scene_generator.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("scene_manager");
    rclcpp::Client<curobo_msgs::srv::GenerateRM>::SharedPtr client = node->create_client<curobo_msgs::srv::GenerateRM>("/generate_rm");

    while (!client->wait_for_service(5s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("scene_manager"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("scene_manager"), "service not available, waiting again...");
    }
    rclcpp::Client<curobo_msgs::srv::SceneGenerator>::SharedPtr client_obj = node->create_client<curobo_msgs::srv::SceneGenerator>("/generate_scene");

    while (!client_obj->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("scene_manager"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("scene_manager"), "service not available, waiting again...");
    }
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("scene_manager"), "Iteration: %d", i + 1);
        auto request = std::make_shared<curobo_msgs::srv::GenerateRM::Request>();
        request->resolution = 0.5;
        request->batch_size = 2804;

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("scene_manager"), "Failed to call service add_two_ints");
        }

        auto request_obj = std::make_shared<curobo_msgs::srv::SceneGenerator::Request>();
        request_obj->nb_object = 20;
        request_obj->max_reach = 1.3;

        auto result_obj = client_obj->async_send_request(request_obj);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result_obj) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("scene_manager"), "Failed to call service add_two_ints");
        }

    }
    // End the timer
    auto end_time = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(rclcpp::get_logger("scene_manager"), "Global time generation: %ld ms", duration);
    rclcpp::shutdown();
    return 0;
}