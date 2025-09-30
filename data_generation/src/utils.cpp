//
// Created by will on 06/06/24.
//

#include "../include/utils.h"



// write a class to add math operation as round_to_decimals

double utils::round_to_decimals(double value, int decimals) {
    double factor = std::pow(10, decimals);
    double res = round(value * factor) / factor;
    if (res >1000) {
        std::cout << "res: " << res << std::endl;
        std::cout << "factor: " << factor << std::endl;
        std::cout << "value: " << value << std::endl;
    }
    return res;
}


bool utils::split_data(std::vector<geometry_msgs::msg::Pose>& data, size_t max_batch_size, std::vector<std::vector<geometry_msgs::msg::Pose>>& batches) {
    if (max_batch_size == 0) {
        std::cerr << "Batch size must be greater than 0." << std::endl;
        return false;
    }
    // Get the number of batch 
    size_t num_batche = std::size(data) / max_batch_size;

    
    // Compute reste and add an absurde value
    size_t reste =   std::size(data) - (num_batche * max_batch_size) ;
    // Absurde value
    geometry_msgs::msg::Pose reste_pose;
    reste_pose.position.x = 1225.0;
    reste_pose.position.y = 1225.0;
    reste_pose.position.z = 1225.0;

    if (reste != 0){
        for (size_t r = 0; r < max_batch_size - reste ; r++){
             data.push_back(reste_pose);
        } 
     }
    num_batche = std::size(data) / max_batch_size;
    batches.clear();
    for (size_t i = 0; i < num_batche; ++i) {
        size_t start_idx = i * max_batch_size;
        size_t end_idx = i * max_batch_size + max_batch_size;
        std::vector<geometry_msgs::msg::Pose> batch(data.begin() + start_idx, data.begin() + end_idx);
        batches.push_back(batch);
    }

    return true;
}


bool utils::save_poses_to_file(const std::string& filename, const std::vector<geometry_msgs::msg::Pose>& poses) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    size_t size = poses.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(size));

    for (const auto& pose : poses) {
        file.write(reinterpret_cast<const char*>(&pose.position.x), sizeof(pose.position.x));
        file.write(reinterpret_cast<const char*>(&pose.position.y), sizeof(pose.position.y));
        file.write(reinterpret_cast<const char*>(&pose.position.z), sizeof(pose.position.z));
        file.write(reinterpret_cast<const char*>(&pose.orientation.x), sizeof(pose.orientation.x));
        file.write(reinterpret_cast<const char*>(&pose.orientation.y), sizeof(pose.orientation.y));
        file.write(reinterpret_cast<const char*>(&pose.orientation.z), sizeof(pose.orientation.z));
        file.write(reinterpret_cast<const char*>(&pose.orientation.w), sizeof(pose.orientation.w));
    }
    return true;
}

bool utils::saveVecToNpz(const std::string& filename, const std::vector<std::array<double, 4>>& data) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    size_t size = data.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(size));

    for (const auto& entry : data) {
        file.write(reinterpret_cast<const char*>(entry.data()), sizeof(double) * 4);
    }

    return true;
}

bool utils::load_poses_from_file(const std::string& filename, std::vector<geometry_msgs::msg::Pose>& poses) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return false;
    }

    size_t size;
    file.read(reinterpret_cast<char*>(&size), sizeof(size));

    poses.resize(size);
    for (auto& pose : poses) {
        file.read(reinterpret_cast<char*>(&pose.position.x), sizeof(pose.position.x));
        file.read(reinterpret_cast<char*>(&pose.position.y), sizeof(pose.position.y));
        file.read(reinterpret_cast<char*>(&pose.position.z), sizeof(pose.position.z));
        file.read(reinterpret_cast<char*>(&pose.orientation.x), sizeof(pose.orientation.x));
        file.read(reinterpret_cast<char*>(&pose.orientation.y), sizeof(pose.orientation.y));
        file.read(reinterpret_cast<char*>(&pose.orientation.z), sizeof(pose.orientation.z));
        file.read(reinterpret_cast<char*>(&pose.orientation.w), sizeof(pose.orientation.w));
    }

    return true;
}