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

    std::shared_ptr<rclcpp::Node>
        node = rclcpp::Node::make_shared("scene_manager");
    rclcpp::Client<curobo_msgs::srv::GenerateRM>::SharedPtr client = node->create_client<curobo_msgs::srv::GenerateRM>("/generate_rm");

    int dataset_size = 0;
    float voxel_size = 0;
    int batch_size = 0;
    float reach_max = 0;
    int obj_max = 0;
    int real_dataset_size = 0;

    node->declare_parameter("dataset_size", 125);
    node->get_parameter("dataset_size", dataset_size);
    node->declare_parameter("voxel_size", 0.5);
    node->get_parameter("voxel_size", voxel_size);
    node->declare_parameter("batch_size", 1000);
    node->get_parameter("batch_size", batch_size);
    node->declare_parameter("reach_max", 1.3);
    node->get_parameter("reach_max", reach_max);
    node->declare_parameter("obj_max", 20);
    node->get_parameter("obj_max", obj_max);

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
    for (int i = 0; i < dataset_size; ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("scene_manager"), "Iteration: %d", i + 1);

        // Add obects
        auto request_obj = std::make_shared<curobo_msgs::srv::SceneGenerator::Request>();
        request_obj->nb_object = obj_max;
        request_obj->max_reach = reach_max;

        auto result_obj = client_obj->async_send_request(request_obj);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result_obj) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("scene_manager"), "Failed to call service add_two_ints");
        }

        // Generate dataset
        auto request = std::make_shared<curobo_msgs::srv::GenerateRM::Request>();
        request->resolution = voxel_size;
        request->batch_size = batch_size;

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            
            RCLCPP_ERROR(rclcpp::get_logger("scene_manager"), "Failed to call service add_two_ints");
        }else{
            auto res = result.get();
            if (res->success){
                real_dataset_size +=1;
            }
        }
        
    }
    // End the timer
    auto end_time = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(rclcpp::get_logger("scene_manager"), "Global time generation: %ld ms, Dataset size: %i", duration, real_dataset_size);
    rclcpp::shutdown();
    return 0;
}