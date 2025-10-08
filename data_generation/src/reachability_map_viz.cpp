//
// Created by will on 06/06/24.
//

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "../include/json.hpp"
#include <std_msgs/msg/string.hpp>
using json = nlohmann::json;
#include <fstream>
#include <string>
#include "../include/utils.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

using json = nlohmann::json;
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <highfive/H5File.hpp>

#include <boost/multi_array.hpp>
#include <highfive/highfive.hpp>

#include <iostream>
using namespace std;
using namespace HighFive;

using namespace std::chrono_literals;

class ReachabilityMapVizualisation : public rclcpp::Node
{
public:
    ReachabilityMapVizualisation()
        : Node("reachability_map_vizualisation_node")
    {
        this->declare_parameter("file_name", "/home/ros2_ws/install/data_generation/share/data_generation/data/master_ik_data0.1.npz");
        this->file_name = this->get_parameter("file_name").as_string();

        // RCLCPP_INFO(node->get_logger(), "msg created");
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        // timer_ = this->create_wall_timer(10s, std::bind(&ReachabilityMapVizualisation::timer_callback, this));
        this->data_file_path = ament_index_cpp::get_package_share_directory("data_generation") + "/data/" + "master_ik_data0.1" + ".h5";
        // mkdir(this->data_file_path.c_str(), 0777);
        this->data_file_ = std::make_shared<HighFive::File>(
            this->data_file_path,
            HighFive::File::ReadWrite | HighFive::File::Create);
        this->convert_rm_npz2h5();
    }

    ~ReachabilityMapVizualisation() {}



    bool convert_rm_npz2h5()
    {
        std::vector<geometry_msgs::msg::Pose> data;

        utils::load_poses_from_file(ament_index_cpp::get_package_share_directory("data_generation") + "/data" + "/master_ik_data" + "0.1" + ".npz", data);

        std::map<std::vector<double>, double> map_rm;

        for (const auto& data_pt : data)
        {
            // get the x, y, z and if joint state valide, add 1 to score 
            std::vector<double> pt = {round(data_pt.position.x*100)/100, round(data_pt.position.y*100)/100, round(data_pt.position.z*100)/100};
            
            map_rm[pt] += 1.0 / 50.0;

        }
        // Save to hdf5
        std::vector<std::array<double, 4>> voxel_map = {};
        int voxel_grid_sizes[3]  = {38, 38, 38};
        double voxel_grid_origin[3] = {-1.5, -1.5, -1.5};
        float resolution = 0.02;
        this->saveToHDF5(map_rm, voxel_map, resolution, voxel_grid_sizes, voxel_grid_origin);


        return true;
    }

      bool saveToHDF5(const std::map<std::vector<double>, double> &data,
                        const std::vector<std::array<double, 4>> &voxel_grid,
                        float voxel_size,
                        int (&voxel_grid_sizes)[3],
                        double (&voxel_grid_origin)[3])
        {
            using namespace HighFive;

            vector<vector<vector<double>>> voxel_grid_data(voxel_grid_sizes[0], 
                                            vector<vector<double>>(voxel_grid_sizes[1], 
                                            vector<double>(voxel_grid_sizes[2], 
                                            1.0))); // initial value is 1.0 (means free voxel)
            
            // 
            double resolution = 0.02;
            double origine = -1.5;
            double max_size = 1.5;

            double rm_size = round(max_size * 2 / resolution);
            
            for (size_t idx = 0; idx < voxel_grid.size(); ++idx) {
            // for(auto it=voxel_grid.begin(); it!=voxel_grid.end(); ++it){
                // int index = std::distance(voxel_grid.begin(), it);
                size_t x = idx / (voxel_grid_sizes[1] * voxel_grid_sizes[2]);
                size_t y = (idx / voxel_grid_sizes[2]) % voxel_grid_sizes[1];
                size_t z = idx % voxel_grid_sizes[2];
                voxel_grid_data[x][y][z] = 1 - voxel_grid[idx][3];
                // rm_data[x][y][z] = data[idx][3];
            }

            // create a rm map with only 0
            vector<vector<vector<double>>> rm_data(rm_size, 
                                            vector<vector<double>>(rm_size, 
                                            vector<double>(rm_size, 
                                            0.0))); // initial value is 0 (means no reach)
            for(const auto &pose : data) {
                int idx = static_cast<int> (round  (((round(pose.first[0] * 100)/100) - origine) / resolution));
                int idy = static_cast<int> (round  (((round(pose.first[1] * 100)/100) - origine) / resolution));
                int idz = static_cast<int> (round  (((round(pose.first[2] * 100)/100) - origine) / resolution));
                rm_data[idx][idy][idz] = pose.second;
                //  RCLCPP_WARN(this->get_logger(), "x: %i, y:%i, z:%i, Data:%f", idx, idy, idz, pose.second);
            }



            std::string dataset_id_s = std::to_string(this->dataset_id);

            // Create the group structure if it does not exist
            auto group = this->data_file_->createGroup("/group/" + dataset_id_s);
            // create voxel map dataset
            std::vector<size_t> dims_voxel_grid{(size_t)voxel_grid_sizes[0], (size_t)voxel_grid_sizes[1], (size_t)voxel_grid_sizes[2]};
            DataSet dataset_voxelgrid = group.createDataSet<double>("voxel_grid", DataSpace(dims_voxel_grid));
            dataset_voxelgrid.write(voxel_grid_data);
            // add voxel map attribut
            dataset_voxelgrid.createAttribute<double>("origine_x", voxel_grid_origin[0]);
            dataset_voxelgrid.createAttribute<double>("origine_y", voxel_grid_origin[1]);
            dataset_voxelgrid.createAttribute<double>("origine_z", voxel_grid_origin[2]);
            dataset_voxelgrid.createAttribute<double>("voxel_size", voxel_size);
            dataset_voxelgrid.createAttribute<double>("voxel_grid_size_x", voxel_grid_sizes[0]);
            dataset_voxelgrid.createAttribute<double>("voxel_grid_size_y", voxel_grid_sizes[1]);
            dataset_voxelgrid.createAttribute<double>("voxel_grid_size_z", voxel_grid_sizes[2]);


            // Now create datasets within these groups
            std::vector<size_t> dims_rm_grid{(size_t)rm_size, (size_t)rm_size, (size_t)rm_size};
            DataSet dataset_data = group.createDataSet<double>("reachability_map", DataSpace(dims_rm_grid));
            dataset_data.write(rm_data);
            // add voxel map attribut
            dataset_data.createAttribute<double>("origine_x", origine);
            dataset_data.createAttribute<double>("origine_y", origine);
            dataset_data.createAttribute<double>("origine_z", origine);
            dataset_data.createAttribute<double>("voxel_size", resolution);
            dataset_data.createAttribute<double>("voxel_grid_size_x", rm_size);
            dataset_data.createAttribute<double>("voxel_grid_size_y", rm_size);
            dataset_data.createAttribute<double>("voxel_grid_size_z", rm_size);
            



            this->dataset_id += 1;
            return true;
        }

private:
    std::string file_name;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    std::shared_ptr<HighFive::File> data_file_;
    std::string data_file_path;
    int dataset_id = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReachabilityMapVizualisation>());

    rclcpp::shutdown();
    return 0;
}
