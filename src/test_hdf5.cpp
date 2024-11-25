#include <rclcpp/rclcpp.hpp>
#include <highfive/highfive.hpp>
#include <vector>

int main(int argc, char * argv[])
{
    // Initialisation du nœud ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hdf5_tester");

    RCLCPP_INFO(node->get_logger(), "Démarrage du nœud hdf5_tester");

    try {
        // Création d'un fichier HDF5 nommé "test.h5"
        HighFive::File file("test.h5", HighFive::File::Overwrite);

        // Données à écrire dans le dataset
        std::vector<int> data = {1, 2, 3, 4, 5};

        // Création et écriture du dataset "dataset_int" dans le fichier
        file.createDataSet<int>("dataset_int", HighFive::DataSpace::From(data)).write(data);

        RCLCPP_INFO(node->get_logger(), "Fichier HDF5 créé et données écrites avec succès.");
    }
    catch (const HighFive::Exception& err) {
        RCLCPP_ERROR(node->get_logger(), "Erreur lors de l'utilisation de HighFive: %s", err.what());
    }

    // Boucle ROS2 (ici, le nœud s'arrête immédiatement après l'écriture)
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
