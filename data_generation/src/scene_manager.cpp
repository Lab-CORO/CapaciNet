#include "rclcpp/rclcpp.hpp"
#include "curobo_msgs/srv/generate_rm.hpp"
#include "curobo_msgs/srv/scene_generator.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"

#include <chrono>
#include <memory>
#include <functional>
#include <future>
#include <random>

using namespace std::chrono_literals;

class SceneManager : public rclcpp::Node
{
public:
    SceneManager() : Node("scene_manager")
    {
        // Declare and get parameters
        this->declare_parameter("dataset_size", 125);
        this->declare_parameter("voxel_size", 0.5);
        this->declare_parameter("batch_size", 1000);
        this->declare_parameter("reach_max", 1.3);
        this->declare_parameter("obj_max", 20);
        this->declare_parameter("use_rosbag_mode", false);
        this->declare_parameter("rosbag_player_name", std::string("rosbag2_player"));
        this->declare_parameter("scene_stabilization_max_delay_sec", 0.5);

        this->get_parameter("dataset_size", dataset_size_);
        this->get_parameter("voxel_size", voxel_size_);
        this->get_parameter("batch_size", batch_size_);
        this->get_parameter("reach_max", reach_max_);
        this->get_parameter("obj_max", obj_max_);
        this->get_parameter("use_rosbag_mode", use_rosbag_mode_);
        this->get_parameter("rosbag_player_name", rosbag_player_name_);
        this->get_parameter("scene_stabilization_max_delay_sec", scene_stabilization_max_delay_sec_);

        // Create service client for generate_rm (always needed)
        client_rm_ = this->create_client<curobo_msgs::srv::GenerateRM>("/generate_rm");

        if (use_rosbag_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "Rosbag mode enabled");

            // Create rosbag control service clients
            std::string pause_service = "/" + rosbag_player_name_ + "/pause";
            std::string resume_service = "/" + rosbag_player_name_ + "/resume";

            client_pause_ = this->create_client<rosbag2_interfaces::srv::Pause>(pause_service);
            client_resume_ = this->create_client<rosbag2_interfaces::srv::Resume>(resume_service);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Generation mode enabled");

            // Create service client for generate_scene (only in generation mode)
            client_scene_ = this->create_client<curobo_msgs::srv::SceneGenerator>("/generate_scene");
        }

        // Create callback group for state machine timer
        timer_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // Start state machine timer at 10Hz (100ms)
        state_machine_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&SceneManager::state_machine_callback, this),
            timer_cb_group_);

        RCLCPP_INFO(this->get_logger(), "SceneManager initialized in WAITING_SERVICES state");
    }

private:
    enum class State
    {
        WAITING_SERVICES,
        IDLE,
        PAUSING_ROSBAG,
        GENERATING_SCENE,
        GENERATING_RM,
        RESUMING_ROSBAG,
        WAITING_SCENE,
        FINISHED
    };

    // Parameters
    int dataset_size_;
    float voxel_size_;
    int batch_size_;
    float reach_max_;
    int obj_max_;
    bool use_rosbag_mode_;
    std::string rosbag_player_name_;
    double scene_stabilization_max_delay_sec_;

    // State
    State state_ = State::WAITING_SERVICES;
    int current_iteration_ = 0;
    int real_dataset_size_ = 0;
    std::chrono::high_resolution_clock::time_point start_time_;

    // State machine timer
    rclcpp::TimerBase::SharedPtr state_machine_timer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    // Service clients
    rclcpp::Client<curobo_msgs::srv::GenerateRM>::SharedPtr client_rm_;
    rclcpp::Client<curobo_msgs::srv::SceneGenerator>::SharedPtr client_scene_;
    rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedPtr client_pause_;
    rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr client_resume_;

    // Pending service futures
    rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedFuture pause_future_;
    rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedFuture resume_future_;
    rclcpp::Client<curobo_msgs::srv::SceneGenerator>::SharedFuture scene_future_;
    rclcpp::Client<curobo_msgs::srv::GenerateRM>::SharedFuture rm_future_;

    // Request in progress flags
    bool pause_request_sent_ = false;
    bool resume_request_sent_ = false;
    bool scene_request_sent_ = false;
    bool rm_request_sent_ = false;

    // Scene stabilization timing
    std::chrono::steady_clock::time_point scene_stabilization_start_;
    bool scene_stabilization_active_ = false;
    double scene_stabilization_current_delay_ = 0.0;  // Current random delay

    // Random number generator for scene stabilization delay
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    std::uniform_real_distribution<double> delay_distribution_{0.0, 1.0};

    // Helper: Convert state to string
    std::string state_to_string(State s)
    {
        switch (s)
        {
            case State::WAITING_SERVICES: return "WAITING_SERVICES";
            case State::IDLE: return "IDLE";
            case State::PAUSING_ROSBAG: return "PAUSING_ROSBAG";
            case State::GENERATING_SCENE: return "GENERATING_SCENE";
            case State::GENERATING_RM: return "GENERATING_RM";
            case State::RESUMING_ROSBAG: return "RESUMING_ROSBAG";
            case State::WAITING_SCENE: return "WAITING_SCENE";
            case State::FINISHED: return "FINISHED";
            default: return "UNKNOWN";
        }
    }

    // Helper: Transition to new state with logging
    void transition_to(State new_state)
    {
        if (state_ == new_state)
        {
            // Avoid logging redundant transitions
            return;
        }

        RCLCPP_INFO(this->get_logger(), "STATE: %s -> %s",
            state_to_string(state_).c_str(),
            state_to_string(new_state).c_str());

        state_ = new_state;
    }

    // Main state machine callback (called at 10Hz)
    void state_machine_callback()
    {
        switch (state_)
        {
            case State::WAITING_SERVICES:
                handle_waiting_services();
                break;
            case State::IDLE:
                handle_idle();
                break;
            case State::PAUSING_ROSBAG:
                handle_pausing_rosbag();
                break;
            case State::GENERATING_SCENE:
                handle_generating_scene();
                break;
            case State::GENERATING_RM:
                handle_generating_rm();
                break;
            case State::RESUMING_ROSBAG:
                handle_resuming_rosbag();
                break;
            case State::WAITING_SCENE:
                handle_waiting_scene();
                break;
            case State::FINISHED:
                handle_finished();
                break;
        }
    }

    // State handler: WAITING_SERVICES
    void handle_waiting_services()
    {
        bool all_services_ready = true;

        // Check generate_rm service
        if (!client_rm_->wait_for_service(0s))
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for /generate_rm service...");
            all_services_ready = false;
        }

        if (use_rosbag_mode_)
        {
            // Check rosbag services
            if (!client_pause_->wait_for_service(0s))
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Waiting for rosbag pause service...");
                all_services_ready = false;
            }
            if (!client_resume_->wait_for_service(0s))
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Waiting for rosbag resume service...");
                all_services_ready = false;
            }
        }
        else
        {
            // Check generate_scene service
            if (!client_scene_->wait_for_service(0s))
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Waiting for /generate_scene service...");
                all_services_ready = false;
            }
        }

        if (all_services_ready)
        {
            RCLCPP_INFO(this->get_logger(), "All services ready. Starting data generation.");
            start_time_ = std::chrono::high_resolution_clock::now();
            transition_to(State::IDLE);
        }
    }

    // State handler: IDLE
    void handle_idle()
    {
        // Check if we've completed all iterations
        if (current_iteration_ >= dataset_size_)
        {
            transition_to(State::FINISHED);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Iteration: %d / %d",
            current_iteration_ + 1, dataset_size_);

        // Initiate next iteration based on mode
        if (use_rosbag_mode_)
        {
            transition_to(State::PAUSING_ROSBAG);
        }
        else
        {
            transition_to(State::GENERATING_SCENE);
        }
    }

    // State handler: PAUSING_ROSBAG
    void handle_pausing_rosbag()
    {
        // Send request if not already sent
        if (!pause_request_sent_)
        {
            auto request = std::make_shared<rosbag2_interfaces::srv::Pause::Request>();
            pause_future_ = client_pause_->async_send_request(request).future.share();
            pause_request_sent_ = true;
            RCLCPP_DEBUG(this->get_logger(), "Sent pause request");
            return;
        }

        // Check if response is ready (non-blocking)
        if (pause_future_.valid())
        {
            auto status = pause_future_.wait_for(std::chrono::seconds(0));
            if (status == std::future_status::ready)
            {
                // Get response (won't block as we know it's ready)
                auto response = pause_future_.get();
                RCLCPP_DEBUG(this->get_logger(), "Rosbag paused");

                // Reset flag for next use
                pause_request_sent_ = false;

                // Transition to next state
                transition_to(State::GENERATING_RM);
            }
            // If not ready, wait for next timer tick
        }
    }

    // State handler: GENERATING_SCENE
    void handle_generating_scene()
    {
        // Send request if not already sent
        if (!scene_request_sent_)
        {
            auto request = std::make_shared<curobo_msgs::srv::SceneGenerator::Request>();
            request->nb_object = obj_max_;
            request->max_reach = reach_max_;
            scene_future_ = client_scene_->async_send_request(request).future.share();
            scene_request_sent_ = true;
            RCLCPP_DEBUG(this->get_logger(), "Sent scene generation request");
            return;
        }

        // Check if response is ready (non-blocking)
        if (scene_future_.valid())
        {
            auto status = scene_future_.wait_for(std::chrono::seconds(0));
            if (status == std::future_status::ready)
            {
                auto response = scene_future_.get();
                if (!response->success)
                {
                    RCLCPP_WARN(this->get_logger(),
                        "Scene generation failed: %s", response->message.c_str());
                }

                // Reset flag for next use
                scene_request_sent_ = false;

                // Transition to next state
                transition_to(State::GENERATING_RM);
            }
            // If not ready, wait for next timer tick
        }
    }

    // State handler: GENERATING_RM
    void handle_generating_rm()
    {
        // Send request if not already sent
        if (!rm_request_sent_)
        {
            auto request = std::make_shared<curobo_msgs::srv::GenerateRM::Request>();
            request->resolution = voxel_size_;
            request->batch_size = batch_size_;
            rm_future_ = client_rm_->async_send_request(request).future.share();
            rm_request_sent_ = true;
            RCLCPP_DEBUG(this->get_logger(), "Sent RM generation request");
            return;
        }

        // Check if response is ready (non-blocking)
        if (rm_future_.valid())
        {
            auto status = rm_future_.wait_for(std::chrono::seconds(0));
            if (status == std::future_status::ready)
            {
                auto response = rm_future_.get();
                if (response->success)
                {
                    real_dataset_size_++;
                    RCLCPP_DEBUG(this->get_logger(), "RM generation successful");
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                        "RM generation failed: %s", response->message.c_str());
                }

                // Increment iteration counter
                current_iteration_++;

                // Reset flag for next use
                rm_request_sent_ = false;

                // Transition based on mode
                if (use_rosbag_mode_)
                {
                    transition_to(State::RESUMING_ROSBAG);
                }
                else
                {
                    transition_to(State::IDLE);
                }
            }
            // If not ready, wait for next timer tick
        }
    }

    // State handler: RESUMING_ROSBAG
    void handle_resuming_rosbag()
    {
        // Send request if not already sent
        if (!resume_request_sent_)
        {
            auto request = std::make_shared<rosbag2_interfaces::srv::Resume::Request>();
            resume_future_ = client_resume_->async_send_request(request).future.share();
            resume_request_sent_ = true;
            RCLCPP_DEBUG(this->get_logger(), "Sent resume request");
            return;
        }

        // Check if response is ready (non-blocking)
        if (resume_future_.valid())
        {
            auto status = resume_future_.wait_for(std::chrono::seconds(0));
            if (status == std::future_status::ready)
            {
                auto response = resume_future_.get();
                RCLCPP_DEBUG(this->get_logger(), "Rosbag resumed");

                // Reset flag for next use
                resume_request_sent_ = false;

                // Generate random delay between 0 and max_delay
                scene_stabilization_current_delay_ = delay_distribution_(gen_) * scene_stabilization_max_delay_sec_;

                // Start scene stabilization delay
                scene_stabilization_start_ = std::chrono::steady_clock::now();
                scene_stabilization_active_ = true;

                RCLCPP_INFO(this->get_logger(),
                    "Waiting %.2fs for scene stabilization (max: %.2fs)",
                    scene_stabilization_current_delay_,
                    scene_stabilization_max_delay_sec_);

                // Transition to waiting state
                transition_to(State::WAITING_SCENE);
            }
            // If not ready, wait for next timer tick
        }
    }

    // State handler: WAITING_SCENE
    void handle_waiting_scene()
    {
        if (!scene_stabilization_active_)
        {
            // Should not happen, but handle gracefully
            RCLCPP_WARN(this->get_logger(),
                "WAITING_SCENE state entered without active stabilization");
            transition_to(State::IDLE);
            return;
        }

        // Check if enough time has elapsed
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - scene_stabilization_start_).count();

        if (elapsed >= scene_stabilization_current_delay_)
        {
            scene_stabilization_active_ = false;
            RCLCPP_DEBUG(this->get_logger(), "Scene stabilization complete (waited %.2fs)", elapsed);
            transition_to(State::IDLE);
        }
        // Otherwise wait for next timer tick
    }

    // State handler: FINISHED
    void handle_finished()
    {
        // This should only be called once
        static bool finished_logged = false;

        if (!finished_logged)
        {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time_).count();

            RCLCPP_INFO(this->get_logger(),
                "Generation complete. Total time: %ld ms, Dataset size: %d / %d",
                duration, real_dataset_size_, dataset_size_);

            finished_logged = true;

            // Stop the timer to prevent further callbacks
            state_machine_timer_->cancel();

            // Shutdown ROS
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SceneManager>();

    // Use MultiThreadedExecutor to follow codebase pattern
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(),
        "Starting SceneManager node, shut down with CTRL-C");
    executor.spin();

    RCLCPP_INFO(node->get_logger(), "SceneManager shutting down.");
    rclcpp::shutdown();
    return 0;
}
