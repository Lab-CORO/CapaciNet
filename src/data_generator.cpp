#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <curobo_msgs/action/data_generation.hpp>
#include <curobo_msgs/srv/generate_rm.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>

#include "utils.h"           // your helper functions, load_poses_from_file, split_data, round_to_decimals, etc.
#include "robot.h"           // your Robot class that performs IK
#include "master_ik_data.h"  // your MasterIkData, PoseOnSphere, etc.

using DataGeneration = curobo_msgs::action::DataGeneration;
using GoalHandleDataGeneration = rclcpp_action::ServerGoalHandle<DataGeneration>;

class DataGeneratorActionServer : public rclcpp::Node
{
public:
    DataGeneratorActionServer()
        : Node("data_generator"),
          robot(this->shared_from_this())
    {
        using namespace std::placeholders;
        
        // Create Action Server
        this->action_server_ = rclcpp_action::create_server<DataGeneration>(
            this,
            "generate_rm",
            std::bind(&DataGeneratorActionServer::handle_goal, this, _1, _2),
            std::bind(&DataGeneratorActionServer::handle_cancel, this, _1),
            std::bind(&DataGeneratorActionServer::handle_accepted, this, _1));
    }

    ~DataGeneratorActionServer() {}

private:
    rclcpp_action::Server<DataGeneration>::SharedPtr action_server_;
    Robot robot;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const DataGeneration::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to generate RM with resolution: %f and batch_size: %d",
                    goal->resolution, goal->batch_size);

        // You can implement some validation logic here
        if (goal->batch_size <= 0 || goal->resolution <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid goal request rejected.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDataGeneration> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDataGeneration> goal_handle)
    {
        // This needs to return quickly, so we spin off a new thread for execution
        std::thread{std::bind(&DataGeneratorActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleDataGeneration> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Starting execution of goal.");
        const auto goal = goal_handle->get_goal();
        
        auto feedback = std::make_shared<DataGeneration::Feedback>();
        auto result = std::make_shared<DataGeneration::Result>();

        float resolution = goal->resolution;
        int batch_size = goal->batch_size;

        // Convert resolution to string for filename
        std::stringstream resolution_string;
        resolution_string << resolution;
        std::string result_str = resolution_string.str();

        std::vector<geometry_msgs::msg::Pose> data;
        try {
            utils::load_poses_from_file(
                ament_index_cpp::get_package_share_directory("data_generation") + "/data/master_ik_data" + result_str + ".npz", 
                data
            );
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load data: %s", e.what());
            result->success = false;
            result->message = "Failed to load data file.";
            goal_handle->abort(result);
            return;
        }

        // Split data into batches
        std::vector<std::vector<geometry_msgs::msg::Pose>> batches;
        utils::split_data(data, batch_size, batches);

        // create a master ik data object
        MasterIkData ik_data;

        // Process each batch
        size_t total_batches = batches.size();
        for (size_t idx = 0; idx < total_batches; ++idx)
        {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal was canceled.");
                result->success = false;
                result->message = "Goal canceled.";
                goal_handle->canceled(result);
                return;
            }

            const auto &batch = batches[idx];
            std::vector<sensor_msgs::msg::JointState> joint_states;
            std::vector<std_msgs::msg::Bool> joint_states_valid;

            robot.get_iks(batch, joint_states, joint_states_valid);

            for (size_t i = 0; i < joint_states.size(); i++)
            {
                if (joint_states_valid[i].data)
                {
                    PoseOnSphere p;
                    p.x = utils::round_to_decimals(batch[i].position.x, 3);
                    p.y = utils::round_to_decimals(batch[i].position.y, 3);
                    p.z = utils::round_to_decimals(batch[i].position.z, 3);
                    p.theta_x = utils::round_to_decimals(batch[i].orientation.x, 6);
                    p.theta_y = utils::round_to_decimals(batch[i].orientation.y, 6);
                    p.theta_z = utils::round_to_decimals(batch[i].orientation.z, 6);
                    p.theta_w = utils::round_to_decimals(batch[i].orientation.w, 6);

                    joint join;
                    join.j1 = joint_states[i].position[0];
                    join.j2 = joint_states[i].position[1];
                    join.j3 = joint_states[i].position[2];
                    join.j4 = joint_states[i].position[3];
                    join.j5 = joint_states[i].position[4];
                    join.j6 = joint_states[i].position[5];

                    std::vector<joint> joints = {join};
                    p.add(joints);

                    ik_data.update_sphere(batch[i].position.x, batch[i].position.y, batch[i].position.z, p);
                }
            }

            // Update feedback after processing each batch
            float progress = static_cast<float>(idx + 1) / static_cast<float>(total_batches);
            feedback->batch_id = 0;
            feedback->batch_nb = 0;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Progress: %.2f%%", progress * 100.0);
        }

        // After all batches are processed, write data to file
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%d_%m_%Y_%H_%M_%S");
        auto date_str = oss.str();

        ik_data.write_data(ament_index_cpp::get_package_share_directory("data_generation") 
                           + "/data/" + date_str + "_" + result_str + ".json");

        // Set result and mark goal as succeeded
        result->success = true;
        result->message = "Reachability map generated successfully.";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
    }
};

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<DataGeneratorActionServer>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }