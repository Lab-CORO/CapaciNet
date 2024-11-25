// this class create a scene to curobo, check the collisions and launch the data generator. it save the scene to a file near the data generator

#include "../include/obstacle_adder.h"

ObstacleAdder::ObstacleAdder() : Node("obstacle_adder")
{
    // Création du client pour le service "add_obstacle"
    client_ = this->create_client<curobo_msgs::srv::AddObject>("/curobo_gen_traj/add_object");

    // Attendre que le service soit disponible
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrompu pendant l'attente du service. Sortie.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service non disponible, nouvelle tentative...");
    }

    // Initialiser le générateur de nombres aléatoires avec une graine basée sur le temps
    std::random_device rd;
    rng_ = std::mt19937(rd());

    // Définir les distributions
    this->num_cubes_dist_ = std::uniform_int_distribution<int>(1, 20); // Nombre de cubes entre 1 et 10
    this->position_dist_ = std::uniform_real_distribution<double>(-2.0, 2.0); // Positions entre -2 et 2
    this->size_dist_ = std::uniform_real_distribution<double>(0.1, 1.0); // Tailles entre 0.1 et 1.0 mètres
}


bool ObstacleAdder::is_in_forbidden_zone(double x, double y, double z, 
                                            double size_x, double size_y, double size_z, 
                                            double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)

{
    // calculer les coordonnées des coins du cube
    double x_cube_min = x - size_x / 2;
    double x_cube_max = x + size_x / 2;
    double y_cube_min = y - size_y / 2;
    double y_cube_max = y + size_y / 2;
    double z_cube_min = z - size_z / 2;
    double z_cube_max = z + size_z / 2;

   // si xa ou xb sont compris entre x1 et x2 ET ya ou yb sont compris entre y1 et y2 ET za ou zb sont compris entre z1 et z2    
    if ((x_min >= x_cube_min && x_min <= x_cube_max) || (x_max >= x_cube_min && x_max <= x_cube_max))
    {
        if ((y_min >= y_cube_min && y_min <= y_cube_max) || (y_max >= y_cube_min && y_max <= y_cube_max))
        {
            if ((z_min >= z_cube_min && z_min <= z_cube_max) || (z_max >= z_cube_min && z_max <= z_cube_max))
            {
                return true;
            }
        }   
    }
    return false; 
}

void ObstacleAdder::add_random_cubes()
{
    // Déterminer un nombre aléatoire de cubes à ajouter
    int num_cubes = this->num_cubes_dist_(rng_);
    RCLCPP_INFO(this->get_logger(), "Ajout de %d cube(s) aléatoire(s).", num_cubes);

    for (int i = 0; i < num_cubes; ++i)
    {
        // Générer une position aléatoire en évitant la zone interdite
        double x, y, z;
        double cube_size;
        do
        {
            x = this->position_dist_(rng_);
            y = this->position_dist_(rng_);
            z = this->position_dist_(rng_);
            // Générer une taille aléatoire pour le cube
            cube_size = this->size_dist_(rng_);
            // print size
            RCLCPP_INFO(this->get_logger(), "Cube size %f", cube_size);
        } while (this->is_in_forbidden_zone(x, y, z, cube_size, cube_size, cube_size, -0.2, 0.2, -0.2, 0.2, -1.45, 1.45));
      
        // Création de la requête
        auto request = std::make_shared<curobo_msgs::srv::AddObject::Request>();

        // Définir le type d'objet en tant que CUBOID (cube)
        request->type = 0;

        // Nommer l'obstacle
        request->name = "cube_" + std::to_string(i);

        // Définir la pose avec les positions aléatoires
        request->pose.position.x = x;
        request->pose.position.y = y;
        request->pose.position.z = z;

        request->pose.orientation.x = 0.0;
        request->pose.orientation.y = 0.0;
        request->pose.orientation.z = 0.0;
        request->pose.orientation.w = 1.0;

        // Définir les dimensions pour le cube
        request->dimensions.x = cube_size;
        request->dimensions.y = cube_size;
        request->dimensions.z = cube_size;

        // Définir une couleur aléatoire
        request->color.r = 1.0;
        request->color.g = 0;
        request->color.b = 0;
        request->color.a = 1.0f; // Opacité totale

        // Envoyer la requête de manière asynchrone
        auto future = client_->async_send_request(request);

        // Attendre le résultat
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Obstacle '%s' ajouté avec succès. Size %f, x = %f, y = %f, z = %f", request->name.c_str(), cube_size, x, y, z);
                
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Échec de l'ajout de l'obstacle '%s' : %s", request->name.c_str(), response->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Échec de l'appel au service add_obstacle pour l'obstacle '%s'", request->name.c_str());
        }
    }
}


int main(int argc, char **argv)
{
    // Initialisation de ROS2
    rclcpp::init(argc, argv);

    // Création du nœud
    auto obstacle_adder_node = std::make_shared<ObstacleAdder>();

    // Appel de la fonction pour ajouter des cubes aléatoires
    obstacle_adder_node->add_random_cubes();

    // Arrêt de ROS2
    rclcpp::shutdown();
    return 0;
}
