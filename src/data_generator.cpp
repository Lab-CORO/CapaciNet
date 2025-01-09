#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "json.hpp"
#include "robot.h"
#include "./master_ik_data.h"
#include "../include/robot.h"
#include "../include/master_ik_data.h"
#include "../include/utils.h"
#include "curobo_msgs/srv/generate_rm.hpp"
#include <curobo_msgs/srv/get_voxel_grid.hpp>
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
            auto callback_generate_rm = [this](const std::shared_ptr<curobo_msgs::srv::GenerateRM::Request> request,
                                               std::shared_ptr<curobo_msgs::srv::GenerateRM::Response> response)
            {
                auto res = data_generation(request->batch_size, request->resolution);
                response->success = res;
                response->message = "Reachability map generated successfully.";
                return response;
            };

            service_ = this->create_service<curobo_msgs::srv::GenerateRM>(
                "generate_rm",
                callback_generate_rm,
                rmw_qos_profile_services_default,
                service_cb_group_);

            client_ik = this->create_client<curobo_msgs::srv::Ik>("/curobo/ik_poses", rmw_qos_profile_services_default,
                                                                  client_cb_group_);
            client_voxel_grid = this->create_client<curobo_msgs::srv::GetVoxelGrid>("/curobo_gen_traj/get_voxel_grid", rmw_qos_profile_services_default,
                                                                                    client_voxel_cb_group_);
        }

        bool data_generation(int batch_size, float resolution)
        {

            std::vector<geometry_msgs::msg::Pose> data;

            std::stringstream resolution_string;
            resolution_string << resolution;              // appending the float value to the streamclass
            std::string result = resolution_string.str(); // converting the float value to string

            utils::load_poses_from_file(ament_index_cpp::get_package_share_directory("data_generation") + "/data" + "/master_ik_data" + result + ".npz", data);

            // split data into batches

            std::vector<std::vector<geometry_msgs::msg::Pose>> batches;
            utils::split_data(data, batch_size, batches);

            // create a master ik data object
            MasterIkData ik_data;

            // iterate all batches

            std::vector<std::array<double, 4>> data_result;
            double sphere[4];
            int data_index = 0;

            for (const auto &batch : batches)
            {
                // send the batch to robot ik
                std::vector<sensor_msgs::msg::JointState> joint_states;
                std::vector<std_msgs::msg::Bool> joint_states_valid;
                // robot.get_iks(batch, joint_states, joint_states_valid);

                auto request = std::make_shared<curobo_msgs::srv::Ik::Request>();

                request->poses = batch;
                // RCLCPP_INFO(node_->get_logger(), "%f",poses[0].position.x);
                auto result_future = client_ik->async_send_request(request);

                std::future_status status = result_future.wait_for(10s); // timeout to guarantee a graceful finish
                if (status == std::future_status::ready)
                {
                    auto res = result_future.get();
                    joint_states = res->joint_states;
                    joint_states_valid = res->joint_states_valid;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed");
                    return false;
                }

                for (size_t i = 0; i < joint_states.size(); i++)
                {
                    if (batch[i].position.x != sphere[0] && batch[i].position.y != sphere[1] && batch[i].position.z != sphere[2])
                    {
                        data_result[data_index][0] = sphere[0];
                        data_result[data_index][1] = sphere[1];
                        data_result[data_index][2] = sphere[2];
                        data_result[data_index][3] = sphere[3];
                        sphere[0] = batch[i].position.x;
                        sphere[1] = batch[i].position.y;
                        sphere[2] = batch[i].position.z;
                        sphere[3] = 0;
                    }
                    // from ik_data get the sphere with key x, y, z
                    if (joint_states_valid[i].data)
                    {
                        sphere[3] += 1 / 50;
                    }
                    else
                    {
                        sphere[3] += 0 / 50;
                    }
                    data_index += 1;
                }
            }
            std::vector<std::array<double, 4>> voxel_map = {};
            this->get_voxel_map(voxel_map);
            this->saveToHDF5(data_result, voxel_map, resolution);

            return true;
        }


        void get_voxel_map(std::vector<std::array<double, 4>> &voxel_grid)
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
                            voxel_grid[index][0] = response->voxel_grid.origin.x + x * response->voxel_grid.resolutions.x;
                            voxel_grid[index][1] = response->voxel_grid.origin.y + y * response->voxel_grid.resolutions.y;
                            voxel_grid[index][2] = response->voxel_grid.origin.z + z * response->voxel_grid.resolutions.z;
                            voxel_grid[index][3] = response->voxel_grid.data[index];
                            
                            ++index;
                        }
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call /get_voxel_grid service.");
                return;
            }

        }

        void saveToHDF5(const std::vector<std::array<double, 4>> &data, const std::vector<std::array<double, 4>> &voxel_grid, float voxel_size)
        {
            using namespace HighFive;

            size_t data_size = data.size();
            size_t voxel_grid_size = voxel_grid.size();
            File data_file(this->data_file_path, File::ReadWrite | File::Truncate);

            std::vector<size_t> dims{data_size, 4};
            std::vector<size_t> dims_voxel_grid{voxel_grid_size, 4};

            std::string dataset_id_s = std::to_string(this->dataset_id);
            DataSet dataset_data = data_file.createDataSet<double>("/group/" + dataset_id_s + "/voxel_map", DataSpace(dims));
            dataset_data.write(data);

            data_file.createDataSet("/group/" + dataset_id_s + "/voxel_size", voxel_size);

            DataSet dataset_voxelgrid = data_file.createDataSet<double>("/group/" + dataset_id_s + "/voxel_grid", DataSpace(dims_voxel_grid));
            dataset_voxelgrid.write(voxel_grid);
        }

    private:
        rclcpp::CallbackGroup::SharedPtr client_cb_group_;
        rclcpp::CallbackGroup::SharedPtr service_cb_group_;
        rclcpp::CallbackGroup::SharedPtr client_voxel_cb_group_;
        
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_ptr_;
        rclcpp::Service<curobo_msgs::srv::GenerateRM>::SharedPtr service_;
        rclcpp::Client<curobo_msgs::srv::Ik>::SharedPtr client_ik;
        rclcpp::Client<curobo_msgs::srv::GetVoxelGrid>::SharedPtr client_voxel_grid;

        std::string data_file_path;
        int dataset_id;
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