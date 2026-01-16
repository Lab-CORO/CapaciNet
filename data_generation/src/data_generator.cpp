#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <iostream>
#include <math.h>
#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include "../include/utils.h"
#include "curobo_msgs/srv/generate_rm.hpp"
#include <curobo_msgs/srv/get_voxel_grid.hpp>
#include "curobo_msgs/srv/ik_batch.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ctime>

#include <highfive/H5File.hpp>

#include <boost/multi_array.hpp>
#include <highfive/highfive.hpp>

#include <iostream>
using namespace std;
using namespace HighFive;

using namespace std::chrono_literals;

namespace cb_data_generator
{
    class DataGenerator : public rclcpp::Node
    {
    public:
        DataGenerator() : Node("client_node")
        {
            // create h45 file name from time
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "%d_%m_%Y_%H_%M_%S");
            auto date_str = oss.str();
            this->data_file_path = ament_index_cpp::get_package_share_directory("data_generation") + "/data/" + date_str + ".h5";

            client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            client_voxel_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            this->dataset_id = 0;
            // Get resolution from ros param
            this->voxel_size = 0;
            this->declare_parameter("voxel_size", 0.5);
            this->get_parameter("voxel_size", voxel_size);
            this->voxel_size = voxel_size;
            std::stringstream resolution_string;
            resolution_string << voxel_size;              // appending the float value to the streamclass
            // Load the reachablity map from the resolution
            utils::load_poses_from_file(ament_index_cpp::get_package_share_directory("data_generation") + "/data" + "/master_ik_data" + resolution_string.str() + ".npz", this->raw_datas);
            for (const auto& data_pt : this->raw_datas) {
                // on prend le risque de la dedondance mais au piure on fera un round sur les coords x y z
                utils::QuantizedPoint3D pt(data_pt.position.x, data_pt.position.y, data_pt.position.z, this->voxel_size);
                this->map_rm[pt] = 0.0;
            }

            auto callback_generate_rm = [this](const std::shared_ptr<curobo_msgs::srv::GenerateRM::Request> request,
                                               std::shared_ptr<curobo_msgs::srv::GenerateRM::Response> response)
            {
                auto res = data_generation(request->batch_size, request->resolution);
                response->success = res;
                response->message = "Reachability map generated successfully.";

                return response;
            };

            this->client_ik = this->create_client<curobo_msgs::srv::IkBatch>("/curobo_ik/ik_batch_poses", rmw_qos_profile_services_default,
                                                                  client_cb_group_);
            client_voxel_grid = this->create_client<curobo_msgs::srv::GetVoxelGrid>("/curobo_ik/get_voxel_grid", rmw_qos_profile_services_default,
                                                                                    client_cb_group_);
            while (!this->client_ik->wait_for_service(std::chrono::seconds(5)))
            {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
                return ;
                }
                RCLCPP_WARN(this->get_logger(), "/client_ik service is not available, retrying...");
            }
            RCLCPP_INFO(this->get_logger(), "/client_ik service is now available.");

            service_ = this->create_service<curobo_msgs::srv::GenerateRM>(
                "generate_rm",
                callback_generate_rm,
                rmw_qos_profile_services_default,
                service_cb_group_);
        }

        ~DataGenerator()
        {
            this->data_file_->flush();
            RCLCPP_INFO(this->get_logger(), "DataGenerator is shutting down.");
        }
        
        bool load_reachability_map(float voxel_size)
        {
            return true;
        }

        bool data_generation(int batch_size, float resolution)
        {
            this->data_file_ = std::make_shared<HighFive::File>(
                this->data_file_path,
                HighFive::File::ReadWrite | HighFive::File::Create);
            auto start_time = std::chrono::high_resolution_clock::now();

            std::vector<geometry_msgs::msg::Pose> data;
            
            // create a copie of this->map_rm to keep a clean object for later 
            auto local_map_rm = this->map_rm;

            // split data into batches
            std::vector<std::vector<geometry_msgs::msg::Pose>> batches;

            utils::split_data(this->raw_datas, batch_size, batches);
            // print batch size
            RCLCPP_WARN(this->get_logger(), "Batch size: %i", batches.size());
            RCLCPP_WARN(this->get_logger(), "Data size: %i", this->raw_datas.size());
            // iterate all batches

            for (const auto &batch : batches)
            {
                // send the batch to robot ik
                std::vector<sensor_msgs::msg::JointState> joint_states;
                std::vector<std_msgs::msg::Bool> joint_states_valid;

                auto request = std::make_shared<curobo_msgs::srv::IkBatch::Request>();

                request->poses = batch;
                auto result_future = this->client_ik->async_send_request(request);

                std::future_status status = result_future.wait_for(10s); // timeout to guarantee a graceful finish
                if (status == std::future_status::ready)
                {
                    auto res = result_future.get();
                    if (!res->success)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Ik batch failed");
                        return false;
                    }
                    joint_states = res->joint_states;
                    joint_states_valid = res->joint_states_valid;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed");
                    return false;
                }

                for (size_t i = 0; i < joint_states.size(); ++i)
                {
                    // get the x, y, z and if joint state valide, add 1 to score 
                    utils::QuantizedPoint3D pt(batch[i].position.x, batch[i].position.y, batch[i].position.z, resolution);
                    
                    if (joint_states_valid[i].data)
                    {
                        local_map_rm[pt] += 1.0 / 50.0;
                    }
                 
                }
            }

            // End the timer
            auto end_time = std::chrono::high_resolution_clock::now();

            // Calculate the elapsed time in milliseconds
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), "RM generation time: %ld ms", duration);

            std::vector<std::array<double, 4>> voxel_map = {};
            int voxel_grid_sizes[3];
            double voxel_grid_origin[3];

            start_time = std::chrono::high_resolution_clock::now();

            this->get_voxel_map(voxel_map, voxel_grid_sizes, voxel_grid_origin);

            // End the timer
            end_time = std::chrono::high_resolution_clock::now();
            // Calculate the elapsed time in milliseconds
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), "Voxel map gen time: %ld ms", duration);

            start_time = std::chrono::high_resolution_clock::now();

            utils::saveToHDF5(local_map_rm, voxel_map, resolution, voxel_grid_sizes, voxel_grid_origin, this->data_file_, this->dataset_id);
            this->dataset_id += 1;
            // End the timer
            end_time = std::chrono::high_resolution_clock::now();
            // Calculate the elapsed time in milliseconds
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), "Save hdf5 time: %ld ms", duration);

            RCLCPP_INFO(this->get_logger(), "Reachability generated !");
            
            this->data_file_->flush();

            return true;
        }

        /**
         * @brief Generate the voxel grid. Here we generate the voxel grid to save it. For AI porpuse, an occupied voxel is 0 and a free voxel is 1.
         */

        void get_voxel_map(std::vector<std::array<double, 4>> &voxel_grid, int (&voxel_grid_sizes)[3], double (&voxel_grid_origin)[3])
        {
            // Create a client for the `/get_voxel_grid` service

            // Wait for the service to be available
            if (!client_voxel_grid->wait_for_service(std::chrono::seconds(5)))
            {
                RCLCPP_WARN(this->get_logger(), "/get_voxel_grid service not available. Retrying...");
                return;
            }

            // Send an empty request
            auto request = std::make_shared<curobo_msgs::srv::GetVoxelGrid::Request>();
            // Call the service asynchronously
            auto result_future = client_voxel_grid->async_send_request(request);
            std::future_status status = result_future.wait_for(10s); // timeout to guarantee a graceful finish
            if (status == std::future_status::ready)
            {
                auto response = result_future.get();

                // Iterate through the voxel grid data and add points for occupied voxels
                int index = 0;
                for (int x = 0; x < (int)response->voxel_grid.size_x; ++x)
                {
                    for (int y = 0; y < (int)response->voxel_grid.size_y; ++y)
                    {
                        for (int z = 0; z < (int)response->voxel_grid.size_z; ++z)
                        {
                            voxel_grid.push_back(std::array<double, 4>{response->voxel_grid.origin.x + x * response->voxel_grid.resolutions.x,
                                                                       response->voxel_grid.origin.y + y * response->voxel_grid.resolutions.y,
                                                                       response->voxel_grid.origin.z + z * response->voxel_grid.resolutions.z,
                                                                       (double)1 - response->voxel_grid.data[index]});
                            ++index;
                        }
                    }
                }
                voxel_grid_sizes[0] = (int)response->voxel_grid.size_x;
                RCLCPP_WARN(this->get_logger(), "voxel size: %d", response->voxel_grid.size_x);
                voxel_grid_sizes[1] = (int)response->voxel_grid.size_y;
                voxel_grid_sizes[2] = (int)response->voxel_grid.size_z;

                voxel_grid_origin[0] = response->voxel_grid.origin.x;
                voxel_grid_origin[1] = response->voxel_grid.origin.y;
                voxel_grid_origin[2] = response->voxel_grid.origin.z;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call /get_voxel_grid service.");
                return;
            }
        }


    private:
        rclcpp::CallbackGroup::SharedPtr client_cb_group_;
        rclcpp::CallbackGroup::SharedPtr service_cb_group_;
        rclcpp::CallbackGroup::SharedPtr client_voxel_cb_group_;

        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_ptr_;
        rclcpp::Service<curobo_msgs::srv::GenerateRM>::SharedPtr service_;
        rclcpp::Client<curobo_msgs::srv::IkBatch>::SharedPtr client_ik;
        rclcpp::Client<curobo_msgs::srv::GetVoxelGrid>::SharedPtr client_voxel_grid;

        std::shared_ptr<HighFive::File> data_file_;
        std::string data_file_path;
        int dataset_id;
        float voxel_size;
        std::map<utils::QuantizedPoint3D, double> map_rm;
        std::vector<geometry_msgs::msg::Pose>raw_datas;
    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<cb_data_generator::DataGenerator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);

    RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
    return 0;
}
