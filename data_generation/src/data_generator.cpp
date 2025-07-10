#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <iostream>
#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include "json.hpp"
#include "robot.h"
// #include "./master_ik_data.h"
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
            // mkdir(this->data_file_path.c_str(), 0777);
            this->data_file_ = std::make_shared<HighFive::File>(
                this->data_file_path,
                HighFive::File::ReadWrite | HighFive::File::Create);

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

            client_ik = this->create_client<curobo_msgs::srv::Ik>("/curobo_ik/ik_batch_poses", rmw_qos_profile_services_default,
                                                                  client_cb_group_);
            client_voxel_grid = this->create_client<curobo_msgs::srv::GetVoxelGrid>("/curobo_ik/get_voxel_grid", rmw_qos_profile_services_default,
                                                                                    client_cb_group_);
            while (!client_ik->wait_for_service(std::chrono::seconds(5)))
            {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
                return ;
                }
                RCLCPP_WARN(this->get_logger(), "/client_ik service is not available, retrying...");
            }
            RCLCPP_INFO(this->get_logger(), "/client_ik service is now available.");
        }

        ~DataGenerator()
        {
            this->data_file_->flush();
            RCLCPP_INFO(this->get_logger(), "DataGenerator is shutting down.");
        }

        bool data_generation(int batch_size, float resolution)
        {

            auto start_time = std::chrono::high_resolution_clock::now();

            std::vector<geometry_msgs::msg::Pose> data;

            std::stringstream resolution_string;
            resolution_string << resolution;              // appending the float value to the streamclass
            std::string result = resolution_string.str(); // converting the float value to string

            utils::load_poses_from_file(ament_index_cpp::get_package_share_directory("data_generation") + "/data" + "/master_ik_data" + result + ".npz", data);
            // split data into batches
            std::vector<std::vector<geometry_msgs::msg::Pose>> batches;
            utils::split_data(data, batch_size, batches);

            // iterate all batches

            std::vector<std::array<double, 4>> data_result;
            data_result.reserve(data.size());

            double sphere[4] = {data[0].position.x, data[0].position.y, data[0].position.z, 0.0};
            for (const auto &batch : batches)
            {
                // send the batch to robot ik
                std::vector<sensor_msgs::msg::JointState> joint_states;
                std::vector<std_msgs::msg::Bool> joint_states_valid;

                auto request = std::make_shared<curobo_msgs::srv::Ik::Request>();

                request->poses = batch;
                auto result_future = client_ik->async_send_request(request);

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

                // data_result.reserve(data_result.size() + data.size());

                for (size_t i = 0; i < joint_states.size(); ++i)
                {
                    // inline distance check
                    double dx = batch[i].position.x - sphere[0];
                    double dy = batch[i].position.y - sphere[1];
                    double dz = batch[i].position.z - sphere[2];
                    double dist_sq = dx * dx + dy * dy + dz * dz;
                    if (dist_sq > (pow(0.001, 2)))
                    { // epsilon^2
                        data_result.push_back({sphere[0], sphere[1], sphere[2], sphere[3]});
                        sphere[0] = batch[i].position.x;
                        sphere[1] = batch[i].position.y;
                        sphere[2] = batch[i].position.z;
                        sphere[3] = 0.0;
                    }
                    if (joint_states_valid[i].data)
                    {
                        sphere[3] += 1.0 / 50.0;
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

            this->saveToHDF5(data_result, voxel_map, resolution, voxel_grid_sizes, voxel_grid_origin);
            // this->save_data(data_result, voxel_map, resolution, voxel_grid_sizes, voxel_grid_origin);
            // End the timer
            end_time = std::chrono::high_resolution_clock::now();
            // Calculate the elapsed time in milliseconds
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), "Save hdf5 time: %ld ms", duration);

            RCLCPP_INFO(this->get_logger(), "Reachability generated !");
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
        bool saveToHDF5(const std::vector<std::array<double, 4>> &data,
                        const std::vector<std::array<double, 4>> &voxel_grid,
                        float voxel_size,
                        int (&voxel_grid_sizes)[3],
                        double (&voxel_grid_origin)[3])
        {
            using namespace HighFive;

            size_t data_size = data.size();
            size_t voxel_grid_size = voxel_grid.size();

            std::vector<size_t> dims{data_size, 4};
            std::vector<size_t> dims_voxel_grid{voxel_grid_size, 4};
            std::string dataset_id_s = std::to_string(this->dataset_id);

            // Create the group structure if it does not exist
            auto group = this->data_file_->createGroup("/group/" + dataset_id_s);
            // create voxel map dataset
            DataSet dataset_voxelgrid = group.createDataSet<double>("voxel_grid", DataSpace(dims_voxel_grid));
            dataset_voxelgrid.write(voxel_grid);
            // add voxel map attribut
            dataset_voxelgrid.createAttribute<double>("origine_x", voxel_grid_origin[0]);
            dataset_voxelgrid.createAttribute<double>("origine_y", voxel_grid_origin[1]);
            dataset_voxelgrid.createAttribute<double>("origine_z", voxel_grid_origin[2]);
            dataset_voxelgrid.createAttribute<double>("voxel_size", voxel_size);
            dataset_voxelgrid.createAttribute<double>("voxel_grid_size_x", voxel_grid_sizes[0]);
            dataset_voxelgrid.createAttribute<double>("voxel_grid_size_y", voxel_grid_sizes[1]);
            dataset_voxelgrid.createAttribute<double>("voxel_grid_size_z", voxel_grid_sizes[2]);


            // Now create datasets within these groups
            DataSet dataset_data = group.createDataSet<double>("reachability_map", DataSpace(dims));
            dataset_data.write(data);
            // add voxel map attribut
            dataset_data.createAttribute<double>("origine_x", voxel_grid_origin[0]);
            dataset_data.createAttribute<double>("origine_y", voxel_grid_origin[1]);
            dataset_data.createAttribute<double>("origine_z", voxel_grid_origin[2]);
            dataset_data.createAttribute<double>("voxel_size", voxel_size);
            dataset_data.createAttribute<double>("voxel_grid_size_x", voxel_grid_sizes[0]);
            dataset_data.createAttribute<double>("voxel_grid_size_y", voxel_grid_sizes[1]);
            dataset_data.createAttribute<double>("voxel_grid_size_z", voxel_grid_sizes[2]);
            



            this->dataset_id += 1;
            return true;
        }

        bool save_data(const std::vector<std::array<double, 4>>& reachability_map, 
                    const std::vector<std::array<double, 4>>& voxel_grid,
                    float voxel_size,
                    int (&voxel_grid_sizes)[3],
                    double (&voxel_grid_origin)[3])
            {
            // create a folder name=id in the main folder
            std::string dataset_id_s = std::to_string(this->dataset_id);
            std::string file_path =  this->data_file_path + "/"  + dataset_id_s;
            if(mkdir(file_path.c_str(), 0777) == -1){
                // ros2 print 
                RCLCPP_WARN(this->get_logger(), "issue at mkdir of data");
            }

            // create reachability map data
            utils::saveVecToNpz(std::string(file_path + "/reachability_map.npz"), reachability_map);
            // create voxel data
            utils::saveVecToNpz(std::string(file_path + "/voxel_grid.npz"), voxel_grid);
            
            // create info.json
            json j;
            j["resolution"] = voxel_size;
            j["voxel_grid_sizes"] = voxel_grid_sizes;
            j["voxel_grid_origin"] = voxel_grid_origin;
            std::ofstream file(file_path + "/info.json");
            file << j.dump(4);
            file.close();
            this->dataset_id += 1;
            return true;
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

        std::shared_ptr<HighFive::File> data_file_;
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