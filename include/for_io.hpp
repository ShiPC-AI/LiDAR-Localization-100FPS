#ifndef _LIDAR_100_UTILS_IO_H_
#define _LIDAR_100_UTILS_IO_H_

#include <iomanip> // for setprecision
#include <cstdlib> // for EXIT_FAILURE
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

void loadKITTIBinToCloudXYZ(const std::string& bin_path, int index, PointCloudPtr& cloud) {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << index; 
    std::string filename = bin_path + ss.str() + ".bin"; 
   
    std::fstream input(filename.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));
    }
    input.seekg(0, std::ios::beg);
    
    while (input.good() && !input.eof()) {
        pcl::PointXYZ point;    
		input.read((char *) &point.x, 3*sizeof(float));
		float intensity;
        input.read((char *) &intensity, sizeof(float));
        cloud->push_back(point);
    } 
}

void loadKITTIPoses(const std::string& filename, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& poses) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));
    }
    std::string line;
    while (getline(ifs, line)) {
        std::vector<double> values(12);   
        std::stringstream ss(line);
        for (int i = 0; i < 12; ++i) {
            ss >> values[i];
        }
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity(); 
        pose(0, 0) = values[0]; pose(0, 1) = values[1]; pose(0, 2) = values[2]; pose(0, 3) = values[3];
        pose(1, 0) = values[4]; pose(1, 1) = values[5]; pose(1, 2) = values[6]; pose(1, 3) = values[7];
        pose(2, 0) = values[8]; pose(2, 1) = values[9]; pose(2, 2) = values[10]; pose(2, 3) = values[11];
        poses.push_back(pose);
    }
    ifs.close();
}

void saveIntegersAsBinary(const std::vector<size_t>& numbers, const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::exit((std::cout << "Failed to save:" << filename << "\n", EXIT_FAILURE));
    }
    // 写入元素数量
    size_t size = numbers.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(size));
    file.write(reinterpret_cast<const char*>(numbers.data()), numbers.size() * sizeof(size_t));
    file.close();
}

void readIntegersFromBinary(const std::string& filename, std::vector<size_t>& numbers) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));
    }
    size_t size;
    file.read(reinterpret_cast<char*>(&size), sizeof(size_t));
    numbers.resize(size);
    file.read(reinterpret_cast<char*>(numbers.data()), size * sizeof(size_t));
    file.close();
}

#endif