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


// void utils::save_vector2d_to_file(const std::string& filename, const std::vector<std::vector<double>>& data) {
//     std::ofstream file(filename, std::ios::binary);
//     if (!file) {
//         std::cerr << "Failed to open file for writing: " << filename << std::endl;
//         return;
//     }
//
//     size_t rows = data.size();
//     size_t cols = rows > 0 ? data[0].size() : 0;
//     file.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
//     file.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
//
//     for (const auto& row : data) {
//         file.write(reinterpret_cast<const char*>(row.data()), cols * sizeof(double));
//     }
// }

// bool utils::load_vector2d_from_file(const std::string& filename, std::vector<std::vector<double>>& data) {
//     std::ifstream file(filename, std::ios::binary);
//     if (!file) {
//         std::cerr << "Failed to open file for reading: " << filename << std::endl;
//         return false;
//     }
//
//     size_t rows, cols;
//     file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
//     file.read(reinterpret_cast<char*>(&cols), sizeof(cols));
//
//     data.resize(rows, std::vector<double>(cols));
//     for (auto& row : data) {
//         file.read(reinterpret_cast<char*>(row.data()), cols * sizeof(double));
//     }
//
//     return true;
// }
//


bool utils::split_data(const std::vector<geometry_msgs::msg::Pose>& data, size_t batch_size, std::vector<std::vector<geometry_msgs::msg::Pose>>& batches) {
    if (batch_size == 0) {
        std::cerr << "Batch size must be greater than 0." << std::endl;
        return false;
    }

    // get the highest divider with a limit (batch_size)
    size_t total_size = data.size();
    int maxDivider = 1;
    for (int i = total_size / 2; i >= 1; --i) { // Vérifie les diviseurs à partir de la moitié de n
        if (total_size % i == 0 && i <= batch_size) {
            maxDivider = i;
            break; 
        }
    }
    

    size_t num_batches = maxDivider;

    batches.clear();
    for (size_t i = 0; i < num_batches; ++i) {
        size_t start_idx = i * batch_size;
        size_t end_idx = std::min(start_idx + batch_size, total_size);
        std::vector<geometry_msgs::msg::Pose> batch(data.begin() + start_idx, data.begin() + end_idx);
        batches.push_back(batch);
    }

    return true;
}

// std::vector<std::vector<std::vector<double>>> utils::split_data(const std::vector<std::vector<double>>& data, size_t batch_size) {
//     std::vector<std::vector<std::vector<double>>> batches;
//     size_t total_size = data.size();
//     size_t num_batches = std::ceil(static_cast<double>(total_size) / batch_size);
//
//     for (size_t i = 0; i < num_batches; ++i) {
//         size_t start_idx = i * batch_size;
//         size_t end_idx = std::min(start_idx + batch_size, total_size);
//         std::vector<std::vector<double>> batch(data.begin() + start_idx, data.begin() + end_idx);
//         batches.push_back(batch);
//     }
//
//     return batches;
// }

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