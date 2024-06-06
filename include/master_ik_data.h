//
// Created by will on 29/03/24.
//

#ifndef SRC_MASTER_IK_DATA_H
#define SRC_MASTER_IK_DATA_H

//#include <jsoncpp/json/json.h>
#include "json.hpp"
using json = nlohmann::json;
#include <iostream>
#include <fstream>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>



struct joint {
    double j1;
    double j2;
    double j3;
    double j4;
    double j5;
    double j6;

    // Méthode existante pour convertir cet objet en un std::vector<double>
    std::vector<double> toVector() const {
        return {j1, j2, j3, j4, j5, j6};
    }

    // Méthode statique pour convertir un std::vector<double> en un objet joint
    static joint fromVector(const std::vector<double>& values) {
        joint j;
        if (values.size() >= 6) { // Assurez-vous qu'il y a suffisamment d'éléments
            j.j1 = values[0];
            j.j2 = values[1];
            j.j3 = values[2];
            j.j4 = values[3];
            j.j5 = values[4];
            j.j6 = values[5];
        } else {
            // Gérez l'erreur ou initialisez j à des valeurs par défaut
            std::cerr << "Erreur: le vecteur ne contient pas suffisamment d'éléments pour convertir en joint." << std::endl;
        }
        return j;
    }
};



    class PoseOnSphere {

    public:
        double x;
        double y;
        double z;
        double theta_x;
        double theta_y;
        double theta_z;
        double theta_w;

        std::vector<joint> joints;

        void to_json(json &j);

        void add(std::vector<joint> &j);

        bool has_joints();

        void remove_joints();

        bool operator<(const PoseOnSphere &other) const {
            if (x < other.x) return true;
            if (x > other.x) return false;
            if (y < other.y) return true;
            if (y > other.y) return false;
            if (z < other.z) return true;
            if (z > other.z) return false;
            if (theta_x < other.theta_x) return true;
            if (theta_x > other.theta_x) return false;
            if (theta_y < other.theta_y) return true;
            if (theta_y > other.theta_y) return false;
            if (theta_z < other.theta_z) return true;
            if (theta_z > other.theta_z) return false;
            if (theta_w < other.theta_w) return true;
            if (theta_w > other.theta_w) return false;
            // Si tous les attributs sont égaux, considérez les PoseOnSphere comme égales pour cet opérateur
            return false;
        }

    };

    class Sphere {

    public:
        double x;
        double y;
        double z;
        std::vector<PoseOnSphere> poses;

        void to_json(json &j);

        void add(PoseOnSphere &p);

        bool has_points();

        bool operator<(const Sphere &other) const {
            if (x < other.x) return true;
            if (x > other.x) return false;
            if (y < other.y) return true;
            if (y > other.y) return false;
            if (z < other.z) return true;
            if (z > other.z) return false;
            // Si x, y et z sont égaux, considérez les Spheres comme égales pour cet opérateur
            return false;
        }
    };


    class MasterIkData: public rclcpp::Node {
/***
 * This class is used to store the data for the master_ik node
 * the first object is compose of a sphere with x,y,z. each sphere is compose of poses (x,y,z, roll, pitch, yaw) and each pose is compose of a vector of 6 doubles
 * A methode save the data in a file json
 * A methode load the data from a file json
 */
    public:
        double resolution;
        double radius;
        int sphere_sample;
        std::vector<Sphere> spheres;
        json json_data;
        std::map<Sphere,std::map<PoseOnSphere, std::vector<joint>>> dict_data;
        // node
        // rclcpp::Node node;

        // construtor
        MasterIkData();
        ~MasterIkData();

        void write_data(std::string filename);

        void load_data(std::string filename);

        void add(Sphere &s);
    };


#endif //SRC_MASTER_IK_DATA_H
