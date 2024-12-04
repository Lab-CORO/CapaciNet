//
// Created by will on 04/04/24.
//

// #include "../include/data_generator.h"
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include "json.hpp"
// #include "../include/robot.h"
#include "./master_ik_data.h"
#include "../include/robot.h"
#include "../include/master_ik_data.h"
#include "../include/utils.h"
#include "curobo_msgs/srv/generate_rm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ctime>
#include "curobo_msgs/srv/ik.hpp"

using json = nlohmann::json;

class DataGenerator : public rclcpp::Node
{
public:
    DataGenerator()
    : Node("data_generator_node")
{
    // this->service_ = this->create_service<curobo_msgs::srv::GenerateRM>(
    //     "generate_rm", std::bind(&DataGenerator::callback_generate_rm, this, std::placeholders::_1, std::placeholders::_2));
         service_ = this->create_service<curobo_msgs::srv::GenerateRM>(
            "generate_rm",
            std::bind(&DataGenerator::callback_generate_rm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            rmw_qos_profile_services_default,
            nullptr//,
            // true  // Indique que la réponse sera différée
        );
}

    void initialize()
    {
        robot = std::make_shared<Robot>(this->shared_from_this());
    }
    

private:
    //    json file
    std::string filename;
    json json_data;
    MasterIkData ik_data;
    MasterIkData ik_data_result;
    // rclcpp::Node node;
    rclcpp::Service<curobo_msgs::srv::GenerateRM>::SharedPtr service_;
    // Robot robot;
    std::shared_ptr<Robot> robot;



    void callback_generate_rm(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<curobo_msgs::srv::GenerateRM::Request> request,
        std::shared_ptr<curobo_msgs::srv::GenerateRM::Response> response)
    {


    float resolution = request->resolution;
    std::vector<geometry_msgs::msg::Pose> data;

    std::stringstream resolution_string;
    resolution_string << resolution;              // appending the float value to the streamclass
    std::string result = resolution_string.str(); // converting the float value to string

    utils::load_poses_from_file(ament_index_cpp::get_package_share_directory("data_generation") + "/data" + "/master_ik_data" + result + ".npz", data);
    RCLCPP_INFO(this->get_logger(), "Loaded %d poses", data.size());   

    // split data into batches
    int batch_size = request->batch_size;
    std::vector<std::vector<geometry_msgs::msg::Pose>> batches;
    utils::split_data(data, batch_size, batches);

    // create a master ik data object
    MasterIkData ik_data;

    auto client_ = this->create_client<curobo_msgs::srv::Ik>("/curobo/ik_poses");

    if (!client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Service /curobo/ik_poses not available");
        response->success = false;
        response->message = "Service not available.";
        // Envoyer la réponse au client
        service_->send_response(request_header, response);
        return;
    }


        // RCLCPP_INFO(this->get_logger(), "Loaded");   
        // send the batch to robot ik
        // std::vector<sensor_msgs::msg::JointState> joint_states;
        // std::vector<std_msgs::msg::Bool> joint_states_valid;
        // robot->get_iks(batch, joint_states, joint_states_valid);


        // Variables partagées pour collecter les résultats
        auto shared_counter = std::make_shared<std::atomic<size_t>>(0);
        auto total_iterations = batches.size();
        auto joint_states_shared = std::make_shared<std::vector<sensor_msgs::msg::JointState>>();
        auto joint_states_valid_shared = std::make_shared<std::vector<std_msgs::msg::Bool>>();

        // Initialiser le compteur
        shared_counter->store(total_iterations);

        // Mutex pour protéger l'accès aux variables partagées
        auto mutex = std::make_shared<std::mutex>();

        // Pour chaque pose, lancer un appel asynchrone au service 'curobo_ik'
           // iterate all batches
        for (const auto &batch : batches)
        {
            auto ik_request = std::make_shared<curobo_msgs::srv::Ik>::Request>();
            ik_request->poses = batch;

            // Envoyer la requête asynchrone
            client_->async_send_request(ik_request,
                [this, request_header, response, shared_counter, joint_states_shared, joint_states_valid_shared, mutex](rclcpp::Client<curobo_msgs::srv::Ik>::SharedFuture result) {
                    try {
                        auto res = result.get();

                        {
                            // Protéger l'accès aux variables partagées
                            std::lock_guard<std::mutex> lock(*mutex);
                            joint_states_shared->push_back(res->joint_states[0]);
                            joint_states_valid_shared->push_back(res->joint_states_valid[0]);
                        }

                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                        {
                            // Protéger l'accès aux variables partagées
                            std::lock_guard<std::mutex> lock(*mutex);
                            // Vous pouvez ajouter des valeurs par défaut ou gérer l'erreur selon vos besoins
                            sensor_msgs::msg::JointState empty_state;
                            std_msgs::msg::Bool valid_flag;
                            valid_flag.data = false;
                            joint_states_shared->push_back(empty_state);
                            joint_states_valid_shared->push_back(valid_flag);
                        }
                    }

                    // Décrémenter le compteur
                    size_t remaining = --(*shared_counter);

                    // Vérifier si tous les appels sont terminés
                    if (remaining == 0) {
                        // Tous les appels sont terminés, préparer la réponse
                        response->joint_states = *joint_states_shared;
                        response->joint_states_valid = *joint_states_valid_shared;
                        response->success = true;
                        response->message = "Successfully obtained all IK solutions.";

                        // Envoyer la réponse au client du service 'generate_rm'
                        service_->send_response(request_header, response);
                    }
                }
            );
        }

        // Ne pas envoyer la réponse ici; elle sera envoyée lorsque tous les appels seront terminés
    }

};










//     void callback_generate_rm(
//     std::shared_ptr<curobo_msgs::srv::GenerateRM::Request> request,
//     std::shared_ptr<curobo_msgs::srv::GenerateRM::Response> response)
// {
//     float resolution = request->resolution;
//     std::vector<geometry_msgs::msg::Pose> data;

//     std::stringstream resolution_string;
//     resolution_string << resolution;              // appending the float value to the streamclass
//     std::string result = resolution_string.str(); // converting the float value to string

//     utils::load_poses_from_file(ament_index_cpp::get_package_share_directory("data_generation") + "/data" + "/master_ik_data" + result + ".npz", data);
//     RCLCPP_INFO(this->get_logger(), "Loaded %d poses", data.size());   

//     // split data into batches
//     int batch_size = request->batch_size;
//     std::vector<std::vector<geometry_msgs::msg::Pose>> batches;
//     utils::split_data(data, batch_size, batches);

//     // create a master ik data object
//     MasterIkData ik_data;

//     // iterate all batches
//     for (const auto &batch : batches)
//     {
//         // RCLCPP_INFO(this->get_logger(), "Loaded");   
//         // send the batch to robot ik
//         std::vector<sensor_msgs::msg::JointState> joint_states;
//         std::vector<std_msgs::msg::Bool> joint_states_valid;
//         // robot->get_iks(batch, joint_states, joint_states_valid);

//         // TEST SERVICE CALL
//         auto client = this->create_client<curobo_msgs::srv::Ik>("/curobo/ik_poses");
//         if (!client->wait_for_service(std::chrono::seconds(5))) {
//                     RCLCPP_ERROR(this->get_logger(), "Service /curobo/ik_poses not available");
//                     response->success = false;
//                     response->message = "Service not available.";
//                     return;
//                 }

//                 auto ik_request = std::make_shared<curobo_msgs::srv::Ik::Request>();
//                 ik_request->poses = batch;

//                 // Envoyez la requête asynchrone
//                 auto future_result = client->async_send_request(ik_request,
//                     [this, response](rclcpp::Client<curobo_msgs::srv::Ik>::SharedFuture result) {
//                         try {
//                             auto res = result.get();
//                             joint_states = res->joint_states;
//                             joint_states_valid = res->joint_states_valid;
//                         } catch (const std::exception &e) {
//                             RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                           
//                         }
//                     }
//                 );


//         // END TEST


//         int nb_valide = 0;
//         for (size_t i = 0; i < joint_states.size(); i++)
//         {
//             // from ik_data get the sphere with key x, y, z
//             if (joint_states_valid[i].data)
//             {
//                 // create a pose in the sphere
//                 PoseOnSphere p;
//                 p.x = utils::round_to_decimals(batch[i].position.x, 3);
//                 p.y = utils::round_to_decimals(batch[i].position.y, 3);
//                 p.z = utils::round_to_decimals(batch[i].position.z, 3);
//                 p.theta_x = utils::round_to_decimals(batch[i].orientation.x, 6);
//                 p.theta_y = utils::round_to_decimals(batch[i].orientation.y, 6);
//                 p.theta_z = utils::round_to_decimals(batch[i].orientation.z, 6);
//                 p.theta_w = utils::round_to_decimals(batch[i].orientation.w, 6);
//                 // if joint[i] is not empty add the joints to the pose
//                 nb_valide++;
//                 // create a joint
//                 joint join;
//                 join.j1 = joint_states[i].position[0];
//                 join.j2 = joint_states[i].position[1];
//                 join.j3 = joint_states[i].position[2];
//                 join.j4 = joint_states[i].position[3];
//                 join.j5 = joint_states[i].position[4];
//                 join.j6 = joint_states[i].position[5];
//                 // add the joint to the pose
//                 std::vector<joint> joints = {join};
//                 p.add(joints);
//                 // add the pose to the sphere
//                 ik_data.update_sphere(batch[i].position.x, batch[i].position.y, batch[i].position.z, p);
//             }
//         }
//         RCLCPP_INFO(this->get_logger(), "Batch size: %d, nb_valide: %d", batch.size(), nb_valide);
//     }
//     // rnd name for file

//     auto t = std::time(nullptr);
//     auto tm = *std::localtime(&t);
//     // std::string date = std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
//     std::ostringstream oss;
//     oss << std::put_time(&tm, "%d_%m_%Y_%H_%M_%S");
//     auto date_str = oss.str();
//     ik_data.write_data(ament_index_cpp::get_package_share_directory("data_generation") + "/data/" + date_str + "_" + result + ".json");


//     response->success = true;
//     response->message = "Reachability map generated successfully.";
       
// }

    // void callback_generate_rm(
    //     const std::shared_ptr<curobo_msgs::srv::GenerateRM::Request> request,
    //     std::shared_ptr<curobo_msgs::srv::GenerateRM::Response> response)
    // {
        
    // }
// };


// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto data_generator = std::make_shared<DataGenerator>();
//     data_generator->initialize();
    
//     rclcpp::spin(data_generator);
//     rclcpp::shutdown();
//     return 0;

    
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataGenerator>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
