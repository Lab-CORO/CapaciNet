#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Inclusions pour Open3D
#include <open3d/Open3D.h>

class MarkerVoxelGrid : public rclcpp::Node
{
public:
    MarkerVoxelGrid()
    : Node("marker_voxel_grid"), voxel_size_(0.05)
    {
        // Création du souscripteur
        subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/visualization_marker_voxel", 10, std::bind(&MarkerVoxelGrid::marker_callback, this, std::placeholders::_1));

        // Initialisation du nuage de points Open3D
        point_cloud_ = std::make_shared<open3d::geometry::PointCloud>();

        // Initialisation du visualiseur dans un thread séparé
        is_visualizer_running_ = true;
        visualizer_thread_ = std::thread(&MarkerVoxelGrid::runVisualizer, this);

        RCLCPP_INFO(this->get_logger(), "MarkerVoxelGrid node has been started.");
    }

    ~MarkerVoxelGrid()
    {
        // Arrêter le visualiseur proprement
        is_visualizer_running_ = false;
        if (visualizer_thread_.joinable())
        {
            visualizer_thread_.join();
        }
    }

private:
    void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Effacer le nuage de points précédent
        point_cloud_->Clear();

        for (const auto& marker : msg->markers)
        {
            // Récupérer la position du marqueur
            double x = marker.pose.position.x;
            double y = marker.pose.position.y;
            double z = marker.pose.position.z;

            // Ajouter le point au nuage de points Open3D
            point_cloud_->points_.emplace_back(x, y, z);

            // Optionnel : Ajouter des couleurs
            point_cloud_->colors_.emplace_back(0.0, 1.0, 0.0); // Vert
        }

        // Créer un Voxel Grid à partir du nuage de points
        voxel_grid_ = open3d::geometry::VoxelGrid::CreateFromPointCloud(*point_cloud_, voxel_size_);

        // Enregistrer le Voxel Grid dans un fichier
        std::string filename = "marker_voxel_grid.ply";
        open3d::io::WriteVoxelGrid(filename, *voxel_grid_);
        RCLCPP_INFO(this->get_logger(), "Voxel Grid saved to %s", filename.c_str());
    }

    void runVisualizer()
    {
        open3d::visualization::Visualizer visualizer;
        visualizer.CreateVisualizerWindow("Marker Voxel Grid Viewer", 800, 600);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (voxel_grid_ != nullptr)
            {
                visualizer.AddGeometry(voxel_grid_);
            }
        }

        while (is_visualizer_running_)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (voxel_grid_ != nullptr)
                {
                    visualizer.UpdateGeometry(voxel_grid_);
                }
            }
            visualizer.PollEvents();
            visualizer.UpdateRender();
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        visualizer.DestroyVisualizerWindow();
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    double voxel_size_;

    // Structures Open3D
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud_;
    std::shared_ptr<open3d::geometry::VoxelGrid> voxel_grid_;

    // Pour la visualisation
    std::thread visualizer_thread_;
    std::mutex mutex_;
    bool is_visualizer_running_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerVoxelGrid>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
