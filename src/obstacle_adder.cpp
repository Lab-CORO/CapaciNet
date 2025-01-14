// this class create a scene to curobo, check the collisions and launch the data generator. it save the scene to a file near the data generator

#include "../include/obstacle_adder.h"

ObstacleAdder::ObstacleAdder() : Node("obstacle_adder")
{
    client_cb_add_obj = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_rm_obj = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    server_scene_generator = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_remove_all = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_collision = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Création du client pour le service "add_obstacle"
    client_add_obj = this->create_client<curobo_msgs::srv::AddObject>("/curobo_ik/add_object", rmw_qos_profile_services_default, client_cb_add_obj);
    client_remove_all_obj = this->create_client<std_srvs::srv::Trigger>("/curobo_ik/remove_all_objects", rmw_qos_profile_services_default, client_cb_remove_all); // trigger

    client_get_collision_dist = this->create_client<curobo_msgs::srv::GetCollisionDistance>("/curobo_ik/get_collision_distance", rmw_qos_profile_services_default, client_cb_collision);
    client_remove_obj = this->create_client<curobo_msgs::srv::RemoveObject>("/curobo_ik/remove_object", rmw_qos_profile_services_default, client_cb_rm_obj);

    // Attendre que le service soit disponible
    while (!client_get_collision_dist->wait_for_service(std::chrono::seconds(1)) &&
           !client_remove_obj->wait_for_service(std::chrono::seconds(1)) &&
           !client_remove_all_obj->wait_for_service(std::chrono::seconds(1)) &&
           !client_add_obj->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrompu pendant l'attente du service. Sortie.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Services non disponible, nouvelle tentative...");
    }

    // Initialiser le générateur de nombres aléatoires avec une graine basée sur le temps
    std::random_device rd;
    rng_ = std::mt19937(rd());

    auto callback_scene_generator = [this](const std::shared_ptr<curobo_msgs::srv::SceneGenerator::Request> request,
                                           std::shared_ptr<curobo_msgs::srv::SceneGenerator::Response> response)
    {
        auto res = add_random_cubes(request->nb_object, request->max_reach);
        response->success = res;
        response->message = "Scene generated";
        return response;
    };

    service_ = this->create_service<curobo_msgs::srv::SceneGenerator>(
        "generate_scene",
        callback_scene_generator,
        rmw_qos_profile_services_default,
        server_scene_generator);
}

/// @brief Verify the cube to add is not in the forbidden zone

bool ObstacleAdder::is_in_forbidden_zone()

{
    // add the obstacle and call distance collision to check if the cube is in the forbidden zone

    auto request = std::make_shared<curobo_msgs::srv::GetCollisionDistance::Request>();

    auto result_future = client_get_collision_dist->async_send_request(request);

    std::future_status status = result_future.wait_for(10s); // timeout to guarantee a graceful finish
    if (status == std::future_status::ready)
    {

        auto response = result_future.get();
        // dillate the response
        if (response->data[0] > -0.05) // >0 in collision so >-0.05 to keep a safe
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Échec de l'appel au service get_collision_dist");
    }

    return false;
}

bool ObstacleAdder::add_random_cubes(int nb_object, float max_reach)
{
    // Définir les distributions
    this->num_cubes_dist_ = std::uniform_int_distribution<int>(2, nb_object);        // Nombre de cubes entre 1 et 10
    this->position_dist_ = std::uniform_real_distribution<double>(-max_reach, max_reach); // Positions entre -2 et 2
    this->size_dist_ = std::uniform_real_distribution<double>(0.1, 0.5);      // Tailles entre 0.1 et 1.0 mètres
    // Supprimer tous les obstacles existants
    auto request_remove = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_remove = client_remove_all_obj->async_send_request(request_remove);
    std::future_status status = future_remove.wait_for(10s); // timeout to guarantee a graceful finish
    if (status == std::future_status::ready)

    {
        auto response = future_remove.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Tous les obstacles ont été supprimés.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Échec de la suppression des obstacles : %s", response->message.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Échec de l'appel au service remove_all_object");
    }
    // Déterminer un nombre aléatoire de cubes à ajouter
    int num_cubes = this->num_cubes_dist_(rng_);
    RCLCPP_INFO(this->get_logger(), "Ajout de %d cube(s) aléatoire(s).", num_cubes);

    for (int i = 0; i < num_cubes; ++i)
    {
        // Générer une position aléatoire en évitant la zone interdite
        double x, y, z;
        double cube_size;
        bool cube_is_accepted = false;
        do
        {
            x = this->position_dist_(rng_);
            y = this->position_dist_(rng_);
            z = this->position_dist_(rng_);
            // Générer une taille aléatoire pour le cube
            // cube_size = this->size_dist_(rng_);
            // print size
            // RCLCPP_INFO(this->get_logger(), "Cube size %f", cube_size);

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
            request->dimensions.x = this->size_dist_(rng_);
            request->dimensions.y = this->size_dist_(rng_);
            request->dimensions.z = this->size_dist_(rng_);

            // Définir une couleur aléatoire
            request->color.r = 1.0;
            request->color.g = 0;
            request->color.b = 0;
            request->color.a = 1.0f; // Opacité totale

            // Envoyer la requête de manière asynchrone
            auto future = client_add_obj->async_send_request(request);

            // Attendre le résultat
            std::future_status status = future.wait_for(10s); // timeout to guarantee a graceful finish
            if (status == std::future_status::ready)

            {
                auto response = future.get();
                if (response->success)
                {
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
            // Check if the cube is accepted
            if (this->is_in_forbidden_zone())
            {
                cube_is_accepted = true;
                RCLCPP_INFO(this->get_logger(), "Obstacle '%s' ajouté avec succès. Size %f, x = %f, y = %f, z = %f", request->name.c_str(), cube_size, x, y, z);
            }
            else
            {
                // remove obj
                auto request = std::make_shared<curobo_msgs::srv::RemoveObject::Request>();
                request->name = "cube_" + std::to_string(i);
                auto future = client_remove_obj->async_send_request(request);

                std::future_status status = future.wait_for(10s); // timeout to guarantee a graceful finish
                if (status == std::future_status::ready)

                {
                    cube_is_accepted = false;
                    auto response = future.get();
                    if (response->success)
                    {
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Échec de la suppression de l'obstacle '%s' : %s", request->name.c_str(), response->message.c_str());
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Échec de l'appel au service add_obstacle pour l'obstacle '%s'", request->name.c_str());
                }
            }
        } while (!cube_is_accepted);
    }
    return true;
}

int main(int argc, char **argv)
{
    // Initialisation de ROS2
    rclcpp::init(argc, argv);

    // Création du nœud
    auto obstacle_adder_node = std::make_shared<ObstacleAdder>();

    // Appel de la fonction pour ajouter des cubes aléatoires
    // obstacle_adder_node->add_random_cubes();

    // TODO TIME OUT FOR THE SERVICE IF ISSUE !!

    // Arrêt de ROS2
    // auto client_node = std::make_shared<cb_data_generator::DataGenerator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(obstacle_adder_node);

    RCLCPP_INFO(obstacle_adder_node->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(obstacle_adder_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
    return 0;
}
