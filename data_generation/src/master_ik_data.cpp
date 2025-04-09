//
// Created by will on 29/03/24.
//

#include "../include/master_ik_data.h"


MasterIkData::MasterIkData(): Node ("master_ik") {
this->resolution = 0.1;
    this->radius = 0.1;
    this->sphere_sample = 0.1;

}

MasterIkData::~MasterIkData() {
    // delete this->node;
}


void MasterIkData::write_data(std::string filename)
{
    // this->node = rclcpp::Node::make_shared("master_ik");
    json j;
    j["resolution"] = resolution;
    j["radius"] = radius;
    j["sphere_sample"] = sphere_sample;
    // add spheres in the json from the dict_sphere map
    std::map<std::string, Sphere>::iterator it;
    // print dict_spher for debug
    // std::cout << "dict_sphere size: " << dict_sphere.size() << std::endl;

    for (auto& [key, sphere] : dict_sphere) {
        json sphere_json;
        sphere.to_json(sphere_json);
        j[key].push_back(sphere_json);
    }

    // print the json for debug
    // std::cout << j.dump(4) << std::endl;
    this->json_data = j;
    std::ofstream o(filename);
    o << j.dump(4) << std::endl;
}



void MasterIkData::load_data(std::string filename) {
    std::ifstream i(filename);
    json j = json::parse(i);

    resolution = j["resolution"];
    radius = j["radius"];
    sphere_sample = j["sphere_sample"];

    for (auto& sphere_json : j["spheres"]) {
        Sphere s;
        s.x = sphere_json["x"];
        s.y = sphere_json["y"];
        s.z = sphere_json["z"];

        for (auto& pose_json : sphere_json["poses"]) {
            PoseOnSphere p;
            p.x = pose_json["x"];
            p.y = pose_json["y"];
            p.z = pose_json["z"];
            p.theta_x = pose_json["theta_x"];
            p.theta_y = pose_json["theta_y"];
            p.theta_z = pose_json["theta_z"];
            p.theta_w = pose_json["theta_w"];

            std::vector<joint> joints;
            for (auto& joint_json : pose_json["joints"]) {
                joint jo;
                // Assigne les valeurs de joint ici
                jo.j1 = joint_json["j1"];
                jo.j2 = joint_json["j2"];
                jo.j3 = joint_json["j3"];
                jo.j4 = joint_json["j4"];
                jo.j5 = joint_json["j5"];
                jo.j6 = joint_json["j6"];
                joints.push_back(jo);
            }
            p.add(joints); // Assurez-vous que cette méthode fonctionne comme prévu
            // Il semble y avoir une intention de mapper PoseOnSphere à ses joints ici
            // Assurez-vous que l'ajout au dict_data est correct
            s.add(p); // Vérifiez que cela ajoute les poses correctement à Sphere
        }

        this->dict_sphere[s.key] = s;
    }
    RCLCPP_INFO(this->get_logger(),"Data loaded");
}





void MasterIkData::add(Sphere &s) {
    // this->spheres.push_back(s);
    std::string sphere_key = "sphere: " + std::to_string(s.x) + "," + std::to_string(s.y) + "," + std::to_string(s.z);
    s.key = sphere_key;
    this->dict_sphere[sphere_key] = {s};
}

bool MasterIkData::update_sphere(double x, double y, double z, PoseOnSphere &new_pose) {
    // this methode get the sphere with the x,y,z
    std::string sphere_key = "sphere: " + std::to_string(utils::round_to_decimals(x, 3)) + ","+
                                std::to_string(utils::round_to_decimals(y, 3)) + "," +
                                std::to_string(utils::round_to_decimals(z, 3));
    new_pose.key = "poseOnSphere:" +
                    std::to_string(x) + "," +
                    std::to_string(y) + "," +
                    std::to_string(z) + "," +
                    std::to_string(new_pose.theta_x) + "," +
                    std::to_string(new_pose.theta_y) + "," +
                    std::to_string(new_pose.theta_z) + "," +
                    std::to_string(new_pose.theta_w);
    if (this->dict_sphere.find(sphere_key) == this->dict_sphere.end()) {
        // create the sphere
        Sphere s;
        s.x = utils::round_to_decimals(x, 3);
        s.y = utils::round_to_decimals(y, 3);
        s.z = utils::round_to_decimals(z, 3);
        s.key = sphere_key;
        s.dict_poses.insert(make_pair(new_pose.key, new_pose));
        this->dict_sphere.insert(make_pair(sphere_key, s));
    }else {
        this->dict_sphere[sphere_key].dict_poses.insert(make_pair(new_pose.key, new_pose));
    }

    return true;

}

void PoseOnSphere::to_json(json &j) {

    j["x"] = this->x;
    j["y"] = this->y;
    j["z"] = this->z;
    j["theta_x"] = this->theta_x;
    j["theta_y"] = this->theta_y;
    j["theta_z"] = this->theta_z;
    j["theta_w"] = this->theta_w;
    // add all joints in the json
    for (const auto& joint : this->joints) {
        json joint_json;
        joint_json["j1"] = joint.j1;
        joint_json["j2"] = joint.j2;
        joint_json["j3"] = joint.j3;
        joint_json["j4"] = joint.j4;
        joint_json["j5"] = joint.j5;
        joint_json["j6"] = joint.j6;
        j["joints"].push_back(joint_json);
    }


}

void PoseOnSphere::add(std::vector<joint> &j) {
    for (int i = 0; i < j.size(); i++) {
        this->joints.push_back(j[i]);
    }
}

bool PoseOnSphere::has_joints() {
    return !this->joints.empty();
}

void PoseOnSphere::remove_joints() {
    this->joints.clear();
}

void Sphere::to_json(json &j) {
    j["x"] = this->x;
    j["y"] = this->y;
    j["z"] = this->z;
    // add all poses in the json from the map
    std::map<std::string, PoseOnSphere>::iterator it;
    for (auto& [key, pose] : dict_poses) {
        json pose_json;
        pose.to_json(pose_json);
        j[key].push_back(pose_json);
    }
}

bool Sphere::has_points() {
    return !this->dict_poses.empty();
}

void Sphere::add(PoseOnSphere &p) {
    std::string poses_key = "poseOnSphere:" + std::to_string(this->x) +
                            "," + std::to_string(this->y) +
                            "," + std::to_string(this->z) +
                            "," + std::to_string(p.theta_x) +
                            "," + std::to_string(p.theta_y) +
                            "," + std::to_string(p.theta_z) +
                            "," + std::to_string(p.theta_w);
    p.key = poses_key;
    this->dict_poses[poses_key] = {p};
}
