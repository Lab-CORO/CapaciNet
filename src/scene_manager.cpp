#include "rclcpp/rclcpp.hpp"
#include "curobo_msgs/srv/scene_generator.hpp"
#include "curobo_msgs/srv/generate_rm.hpp"

using namespace std::chrono_literals;

namespace cd_scene_manager
{
    class SceneManager : public rclcpp::Node
    {
    public:
        SceneManager()
            : Node("scene_manager")
        {
            RCLCPP_INFO(this->get_logger(), "Scene Manager Node Initialized");
            client_obj_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            client_datagen_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // Create clients for the services
            scene_client_ = this->create_client<curobo_msgs::srv::SceneGenerator>("/generate_scene", rmw_qos_profile_services_default,
                                                                                  client_obj_cb_group_);
            rm_client_ = this->create_client<curobo_msgs::srv::GenerateRM>("/generate_rm", rmw_qos_profile_services_default,
                                                                           client_datagen_cb_group_);

            // Wait indefinitely for /generate_scene service
            RCLCPP_INFO(this->get_logger(), "Waiting for /generate_scene service...");
            while (!scene_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "/generate_scene service is not available, retrying...");
            }
            RCLCPP_INFO(this->get_logger(), "/generate_scene service is now available.");

            // Wait indefinitely for /generate_rm service
            RCLCPP_INFO(this->get_logger(), "Waiting for /generate_rm service...");
            while (!rm_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "/generate_rm service is not available, retrying...");
            }
            RCLCPP_INFO(this->get_logger(), "/generate_rm service is now available.");

            RCLCPP_INFO(this->get_logger(), "All services are available. Starting iterations.");

            this->generate_dataset();
        }

        bool generate_dataset()
        {
            // Loop to call services 100 times
            for (int i = 0; i < 100; ++i)
            {
                RCLCPP_INFO(this->get_logger(), "Iteration: %d", i + 1);

                // Call /generate_scene service
                if (!callGenerateScene(50, 1.3))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to call /generate_scene service");
                }

                // Call /generate_rm service
                if (!callGenerateRm(0.5, 2500))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to call /generate_rm service");
                }
            }

            RCLCPP_INFO(this->get_logger(), "Completed 100 iterations");
            return true;
        }

    private:
        rclcpp::CallbackGroup::SharedPtr client_obj_cb_group_;
        rclcpp::CallbackGroup::SharedPtr client_datagen_cb_group_;
        rclcpp::Client<curobo_msgs::srv::SceneGenerator>::SharedPtr scene_client_;
        rclcpp::Client<curobo_msgs::srv::GenerateRM>::SharedPtr rm_client_;

        bool callGenerateScene(int nb_object, float max_reach)
        {
            if (nb_object > 100)
            {
                RCLCPP_ERROR(this->get_logger(), "Too much obstacles in the scene max 100 or modify curobo max obb");
                return false;
            }
            auto request = std::make_shared<curobo_msgs::srv::SceneGenerator::Request>();
            request->nb_object = nb_object; // Replace with actual request parameters
            request->max_reach = max_reach;

            auto result_future = scene_client_->async_send_request(request);
            
            std::future_status status = result_future.wait_for(10s); // timeout to guarantee a graceful finish
            if (status == std::future_status::ready)
            {
                auto response = result_future.get();
                if (response->success != true)
                {
                    RCLCPP_INFO(this->get_logger(), "Failed generate a scene");
                    return false;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Successfully generate a scene");
            return true;


        }

        bool callGenerateRm(float resolution, int batch_size)
        {
            auto request = std::make_shared<curobo_msgs::srv::GenerateRM::Request>();
            request->resolution = resolution;
            request->batch_size = batch_size;

            auto result_future = rm_client_->async_send_request(request);
            std::future_status status = result_future.wait_for(10s); // timeout to guarantee a graceful finish
            if (status == std::future_status::ready)
            {
                auto response = result_future.get();
                if (response->success != true)
                {
                    RCLCPP_INFO(this->get_logger(), "Failed generate RM");
                    return false;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Successfully called /generate_rm");
            return true;
        }
    };
}
int main(int argc, char **argv)
{
    // rclcpp::init(argc, argv);
    // auto node = SceneManager();
    // node.generate_dataset();
    // // rclcpp::spin(node);
    // rclcpp::shutdown();
    // return 0;

    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<cd_scene_manager::SceneManager>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);

    RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
    // client_node.();
    executor.spin();
    RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
    return 0;
}
