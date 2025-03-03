#include <memory>
#include <vector>
#include <string>
#include <cstring>  // std::memcpy

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <HighFive/HighFive.hpp>  // Bibliothèque HighFive

class VoxelPublisher : public rclcpp::Node
{
public:
  VoxelPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("voxel_publisher", options)
  {
    // Paramètre ou chemin fixe vers le fichier .h5
    this->declare_parameter<std::string>("filename", "voxel_data.h5");
    filename_ = this->get_parameter("filename").as_string();

    // Lecture du fichier et calcul des voxels occupés
    loadVoxelData(filename_);

    // Publisher ROS2
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_occupancy", 10);

    // Publier tous les 2 s
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&VoxelPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "VoxelPublisher node started.");
  }

private:
  void loadVoxelData(const std::string & filename)
  {
    // Ouvre le fichier en lecture seule
    HighFive::File file(filename, HighFive::File::ReadOnly);

    //=== 1) Lecture de la grille 3D : /voxel_map/data ===
    auto dataset = file.getDataSet("/voxel_map/data");
    voxel_data_ = dataset.read<std::vector<int>>();  // Récupère tout dans un std::vector<int>

    // Dimensions : (nx_, ny_, nz_)
    auto dims = dataset.getDimensions();
    if (dims.size() != 3) {
      throw std::runtime_error("Le dataset /voxel_map/data n'est pas 3D !");
    }
    nx_ = dims[0];
    ny_ = dims[1];
    nz_ = dims[2];

    //=== 2) Lecture de l'origine (x, y, z) ===
    origin_[0] = file.getDataSet("/voxel_map/origine/x").read<float>();
    origin_[1] = file.getDataSet("/voxel_map/origine/y").read<float>();
    origin_[2] = file.getDataSet("/voxel_map/origine/z").read<float>();

    //=== 3) Lecture de la taille de voxel (x, y, z) ===
    voxel_size_[0] = file.getDataSet("/voxel_map/voxel_size/x").read<float>();
    voxel_size_[1] = file.getDataSet("/voxel_map/voxel_size/y").read<float>();
    voxel_size_[2] = file.getDataSet("/voxel_map/voxel_size/z").read<float>();

    RCLCPP_INFO(
      this->get_logger(),
      "HDF5 loaded: grid=%lux%lux%lu, origin=(%.2f,%.2f,%.2f), voxel_size=(%.3f,%.3f,%.3f)",
      nx_, ny_, nz_,
      origin_[0], origin_[1], origin_[2],
      voxel_size_[0], voxel_size_[1], voxel_size_[2]);

    // On calcule les positions (x, y, z) pour les voxels occupés
    computeOccupiedVoxels();
  }

  void computeOccupiedVoxels()
  {
    points_.clear();
    points_.reserve(voxel_data_.size() * 3); // reserve max (même si on en aura moins)

    // Parcours de la grille
    for (size_t i = 0; i < nx_; i++) {
      for (size_t j = 0; j < ny_; j++) {
        for (size_t k = 0; k < nz_; k++) {
          // Index linéaire
          size_t idx = i * (ny_ * nz_) + j * nz_ + k;
          int val = voxel_data_[idx];
          // 0 => Occupé
          if (val == 0) {
            float x = origin_[0] + i * voxel_size_[0];
            float y = origin_[1] + j * voxel_size_[1];
            float z = origin_[2] + k * voxel_size_[2];
            // Stockage en (x, y, z)
            points_.push_back(x);
            points_.push_back(y);
            points_.push_back(z);
          }
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "Found %zu occupied voxels.", points_.size() / 3);
  }

  void timerCallback()
  {
    // Publier le nuage de points
    if (points_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No occupied voxels to publish.");
      return;
    }

    auto cloud_msg = createPointCloud2(points_);
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = "map";  // Ajuster le frame si nécessaire

    publisher_->publish(cloud_msg);
    RCLCPP_INFO(this->get_logger(), "Published %u points.", cloud_msg.width);
  }

  // Construction du sensor_msgs::msg::PointCloud2 (x, y, z en float32)
  sensor_msgs::msg::PointCloud2 createPointCloud2(const std::vector<float> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    size_t npoints = points.size() / 3;

    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(npoints);
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;
    // 3 float32 => 12 octets par point
    cloud_msg.point_step = 12;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    // Champs x, y, z (float32)
    sensor_msgs::msg::PointField f1, f2, f3;
    f1.name = "x"; f1.offset = 0;  f1.datatype = sensor_msgs::msg::PointField::FLOAT32; f1.count = 1;
    f2.name = "y"; f2.offset = 4;  f2.datatype = sensor_msgs::msg::PointField::FLOAT32; f2.count = 1;
    f3.name = "z"; f3.offset = 8;  f3.datatype = sensor_msgs::msg::PointField::FLOAT32; f3.count = 1;
    cloud_msg.fields = {f1, f2, f3};

    // On alloue la taille binaire
    cloud_msg.data.resize(npoints * cloud_msg.point_step);

    // Copie binaire (x, y, z)
    uint8_t* ptr = cloud_msg.data.data();
    for (size_t i = 0; i < npoints; i++) {
      float x = points[i*3 + 0];
      float y = points[i*3 + 1];
      float z = points[i*3 + 2];
      std::memcpy(ptr + 0, &x, 4);
      std::memcpy(ptr + 4, &y, 4);
      std::memcpy(ptr + 8, &z, 4);
      ptr += 12;
    }

    return cloud_msg;
  }

private:
  std::string filename_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Grille 3D lue dans /voxel_map/data
  std::vector<int> voxel_data_;   
  size_t nx_{0}, ny_{0}, nz_{0};
  float origin_[3] = {0.f, 0.f, 0.f};
  float voxel_size_[3] = {1.f, 1.f, 1.f};

  // Positions (x, y, z) des voxels occupés
  std::vector<float> points_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
