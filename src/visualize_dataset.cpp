#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <highfive/H5File.hpp>

#include <boost/multi_array.hpp>
#include <highfive/highfive.hpp>

using namespace std;
using namespace HighFive;

class VoxelMapPublisher : public rclcpp::Node
{
public:
    VoxelMapPublisher()
        : Node("voxel_map_publisher")
    {
        // Declare and get the path to the HDF5 file as a parameter
        this->declare_parameter<std::string>("h5_file_path", "/home/ros2_ws/install/data_generation/share/data_generation/data/16_01_2025_07_07_09.h5");
        std::string h5_file_path;
        this->get_parameter("h5_file_path", h5_file_path);

        if (h5_file_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "HDF5 file path is not provided!");
            rclcpp::shutdown();
            return;
        }

        try
        {
            // Read data from the HDF5 file
            this->dataset_id = 0;
            read_hdf5(h5_file_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read HDF5 file: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // Publisher for the voxel map visualization
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("voxel_map", 10);

        // Publish the voxel map as a Marker message
        publish_voxel_map();
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    std::vector<std::array<double, 4>> voxel_map_;        // Voxel map data
    std::vector<std::array<double, 4>> reachability_map_; // reachability map data
    float voxel_resolution_;
    double voxel_map_origine_[3]; // Voxel resolution
    int dataset_id;
    int voxel_grid_size[3];

    void read_hdf5(const std::string &file_path)
    {
        using namespace HighFive;

        // Open the HDF5 file
        File file(file_path, File::ReadOnly);

        std::string dataset_id_s = std::to_string(this->dataset_id);
        // Read the voxel_map dataset
        DataSet voxel_map_dataset = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/data");
        voxel_map_dataset.read(this->voxel_map_);
        // Debug
        // for (size_t i = 0; i < voxel_map_.size(); ++i)
        // {
        //     std::ostringstream oss;
        //     oss << "voxel_grid[" << i << "]: ";
        //     for (size_t j = 0; j < voxel_map_[i].size(); ++j)
        //     {
        //         oss << voxel_map_[i][j] << " ";
        //     }
        //     RCLCPP_INFO(this->get_logger(), oss.str().c_str());
        // }

        // get voxel grid size
        DataSet voxel_size_x_dataset = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/x");
        voxel_size_x_dataset.read(voxel_grid_size[0]);
        DataSet voxel_size_y_dataset = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/y");
        voxel_size_y_dataset.read(voxel_grid_size[1]);
        DataSet voxel_size_z_dataset = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/z");
        voxel_size_z_dataset.read(voxel_grid_size[2]);
        // ros print voxel grid size
        RCLCPP_WARN(this->get_logger(), "Voxel grid size: x: %d, y:%d, z:%d", voxel_grid_size[0], voxel_grid_size[1], voxel_grid_size[2]);

        // voxel map origine
        DataSet voxel_map_origine_x = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/origine/x");
        DataSet voxel_map_origine_y = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/origine/y");
        DataSet voxel_map_origine_z = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/origine/z");
        voxel_map_origine_x.read(voxel_map_origine_[0]);
        voxel_map_origine_y.read(voxel_map_origine_[1]);
        voxel_map_origine_z.read(voxel_map_origine_[2]);

        // Read the reachability_map dataset

        DataSet reachability_map_dataset = file.getDataSet("/group/" + dataset_id_s + "/reachability_map");
        reachability_map_dataset.read(reachability_map_);

        // Read the voxel_resolution dataset
        DataSet voxel_resolution_dataset = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_resolutiion");
        voxel_resolution_dataset.read(voxel_resolution_);
        // ros print voxel resolution
        RCLCPP_WARN(this->get_logger(), "Voxel resolution: %f", voxel_resolution_);
    }

    void publish_voxel_map()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_0"; // Adjust frame ID as needed
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "voxel_grid";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set marker scale
        marker.scale.x = voxel_resolution_;
        marker.scale.y = voxel_resolution_;
        marker.scale.z = voxel_resolution_;
        int index = 0;
        // Populate marker points from the voxel map
        for (unsigned int x = 0; x < voxel_grid_size[0]; x++)
        {
            for (unsigned int y = 0; y < voxel_grid_size[1]; y++)
            {
                for (unsigned int z = 0; z < voxel_grid_size[2]; z++)
                {

                    if ((double)this->voxel_map_[index][3] == 0)
                    { // Check if the voxel is occupied
                        geometry_msgs::msg::Point point;
                        point.x = x * voxel_resolution_ + voxel_map_origine_[0];
                        point.y = y * voxel_resolution_ + voxel_map_origine_[1];
                        point.z = z * voxel_resolution_ + voxel_map_origine_[2];
                        // Set marker default color (transparent green)
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        marker.color.a = 0.5;
                        marker.points.push_back(point);
                    }
                    else
                    {
                        // geometry_msgs::msg::Point point;
                        // point.x = x * voxel_resolution_ + voxel_map_origine_[0];
                        // point.y = y * voxel_resolution_ + voxel_map_origine_[1];
                        // point.z = z * voxel_resolution_ + voxel_map_origine_[2];
                        // // Set marker default color (transparent green)
                        // marker.color.r = 0.0;
                        // marker.color.g = 1.0;
                        // marker.color.b = 0.0;
                        // marker.color.a = 0.1;
                        // marker.points.push_back(point);
                    }
                    index++;
                }
            }
        }

        publisher_->publish(marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoxelMapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
