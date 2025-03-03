// this file is a class to read and write h5 files for this project
#include "../include/hdf5_manager.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include <visualization_msgs/msg/marker.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// // #include <highfive/H5File.hpp>

// #include <boost/multi_array.hpp>
// #include <highfive/highfive.hpp>

using namespace HighFive;

// methods
DatasetH5Manager::DatasetH5Manager(const std::string &file_path)
{

    this->data_file_ = std::make_shared<HighFive::File>(
        file_path,
        HighFive::File::ReadWrite | HighFive::File::Create);
}

// write file
void DatasetH5Manager::saveToHDF5(const std::vector<std::array<double, 4>> &data, const std::vector<std::array<double, 4>> &voxel_grid, float voxel_size, int (&voxel_grid_sizes)[3], double (&voxel_grid_origin)[3])
{

    // size_t data_size = data.size();
    // size_t voxel_grid_size = voxel_grid.size();

    // std::vector<size_t> dims{data_size, 4};
    // std::vector<size_t> dims_voxel_grid{voxel_grid_size, 4};

    // std::string dataset_id_s = std::to_string(dataset_id);
    // DataSet dataset_data = this->data_file_->createDataSet<double>("/group/" + dataset_id_s + "/reachability_map", DataSpace(dims));
    // dataset_data.write(data);

    // // save voxel grid origine
    // this->data_file_->createDataSet<double>("/group/" + dataset_id_s + "/voxel_map/origine/x", voxel_grid_origin[0]);
    // this->data_file_->createDataSet<double>("/group/" + dataset_id_s + "/voxel_map/origine/y", voxel_grid_origin[1]);
    // this->data_file_->createDataSet<double>("/group/" + dataset_id_s + "/voxel_map/origine/z", voxel_grid_origin[2]);

    // this->data_file_->createDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_resolutiion", voxel_size);
    // this->data_file_->createDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/x", voxel_grid_sizes[0]);
    // this->data_file_->createDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/y", voxel_grid_sizes[1]);
    // this->data_file_->createDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/z", voxel_grid_sizes[2]);

    // DataSet dataset_voxelgrid = this->data_file_->createDataSet<double>("/group/" + dataset_id_s + "/voxel_map/data", DataSpace(dims_voxel_grid));
    // dataset_voxelgrid.write(voxel_grid);
    // this->dataset_id += 1;
}
// read h5 file from path with highfive
void DatasetH5Manager::read_h5_file(std::string file_path)
{
    // Open the HDF5 file
    // File file(file_path, File::ReadOnly);

    // std::string dataset_id_s = std::to_string(this->dataset_id);
    // // Read the voxel_map dataset
    // DataSet voxel_map_dataset = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/data");
    // voxel_map_dataset.read(this->voxel_map_);
    // // Debug

    // // get voxel grid size
    // DataSet voxel_size_x_dataset = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/x");
    // voxel_size_x_dataset.read(voxel_grid_size[0]);
    // DataSet voxel_size_y_dataset = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/y");
    // voxel_size_y_dataset.read(voxel_grid_size[1]);
    // DataSet voxel_size_z_dataset = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/z");
    // voxel_size_z_dataset.read(voxel_grid_size[2]);
    // // ros print voxel grid size
    // RCLCPP_WARN(this->get_logger(), "Voxel grid size: x: %d, y:%d, z:%d", voxel_grid_size[0], voxel_grid_size[1], voxel_grid_size[2]);

    // // voxel map origine
    // DataSet voxel_map_origine_x = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/origine/x");
    // DataSet voxel_map_origine_y = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/origine/y");
    // DataSet voxel_map_origine_z = file.getDataSet("/group/" + dataset_id_s + "/voxel_map/origine/z");
    // voxel_map_origine_x.read(voxel_map_origine_[0]);
    // voxel_map_origine_y.read(voxel_map_origine_[1]);
    // voxel_map_origine_z.read(voxel_map_origine_[2]);

    // // Read the reachability_map dataset

    // DataSet reachability_map_dataset = file.getDataSet("/group/" + dataset_id_s + "/reachability_map");
    // reachability_map_dataset.read(reachability_map_);
}

// get voxel map
std::vector<std::array<double, 4>> DatasetH5Manager::get_voxel_map(int dataset_id)
{
    
    std::vector<std::array<double, 4>> voxel_map;
    std::string dataset_id_s = std::to_string(dataset_id);
    auto voxel_map_dataset = this->data_file_->getDataSet("/group/" + dataset_id_s + "/voxel_map/data");
    voxel_map_dataset.read(voxel_map);
    return voxel_map;
}

// get voxel parameters
std::array<double, 3> DatasetH5Manager::get_voxel_parameters(int dataset_id)
{
    std::array<double, 3> voxel_grid_size;
    std::string dataset_id_s = std::to_string(dataset_id);

    // get voxel grid size
    DataSet voxel_size_x_dataset = this->data_file_->getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/x");
    DataSet voxel_size_y_dataset = this->data_file_->getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/y");
    DataSet voxel_size_z_dataset = this->data_file_->getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_size/z");

    voxel_size_x_dataset.read(voxel_grid_size[0]);
    voxel_size_y_dataset.read(voxel_grid_size[1]);
    voxel_size_z_dataset.read(voxel_grid_size[2]);

    return voxel_grid_size;
}

// voxel map origine
std::array<double, 3> DatasetH5Manager::get_voxelmap_origine(int dataset_id)
{

    std::array<double, 3> voxel_map_origine;
    std::string dataset_id_s = std::to_string(dataset_id);

    DataSet voxel_map_origine_x = this->data_file_->getDataSet("/group/" + dataset_id_s + "/voxel_map/origine/x");
    DataSet voxel_map_origine_y = this->data_file_->getDataSet("/group/" + dataset_id_s + "/voxel_map/origine/y");
    DataSet voxel_map_origine_z = this->data_file_->getDataSet("/group/" + dataset_id_s + "/voxel_map/origine/z");

    voxel_map_origine_x.read(voxel_map_origine[0]);
    voxel_map_origine_y.read(voxel_map_origine[1]);
    voxel_map_origine_z.read(voxel_map_origine[2]);

    return voxel_map_origine;
}

// get the number of dataset id in a group
int DatasetH5Manager::get_dataset_size()
{
    std::string group_name = "/group/";
    int dataset_size = this->data_file_->getGroup(group_name).getNumberObjects();
    return dataset_size;
}

double DatasetH5Manager::get_resolution(int dataset_id)
{

    std::string dataset_id_s = std::to_string(dataset_id);
    DataSet resolution_dataset = this->data_file_->getDataSet("/group/" + dataset_id_s + "/voxel_map/voxel_resolutiion");
    double resolution;
    resolution_dataset.read(resolution);
    return resolution;
}

// get reachability map
std::vector<std::array<double, 4>> DatasetH5Manager::get_reachability_map(int dataset_id)
{
    std::vector<std::array<double, 4>> reachability_map;
    std::string dataset_id_s = std::to_string(dataset_id);

    DataSet reachability_map_dataset = this->data_file_->getDataSet("/group/" + dataset_id_s + "/reachability_map");
    reachability_map_dataset.read(reachability_map);
    return reachability_map;
}
