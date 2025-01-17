#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include "for_time.hpp"
#include "for_cloud.hpp"
#include "for_desc.hpp"
#include "for_io.hpp"

#include "pcl/io/pcd_io.h"
#include <pcl/filters/extract_indices.h> 
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

void simulateOrientationWithinMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_map, const pcl::PointXYZ& cnt_pt,
    int rows, int cols, int length, int theta_num, std::vector<Eigen::MatrixXi>& descs) {
    
    if (cloud_map->empty()) {
        Eigen::MatrixXi desc = Eigen::MatrixXi::Zero(1, rows * cols);
        descs.resize(theta_num, desc); // Fill with descs with zero
        return;
    }
    
    float reso_theta = M_PI * 2.0 / theta_num;
    float cnt_pt_x = cnt_pt.x;
    float cnt_pt_y = cnt_pt.y;

    // preallocate matrix 
    Eigen::MatrixXi desc(1, rows * cols);  
    for (int i = 0; i < theta_num; ++i) {
        Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
        double theta = i * reso_theta;
        double theta_cos = std::cos(theta);
        double theta_sin = std::sin(theta);        

        trans(0, 0) = theta_cos; 
        trans(0, 1) = -theta_sin; 
        trans(1, 0) = theta_sin; 
        trans(1, 1) = theta_cos;
        
        double cos_offset = 1 - theta_cos; 
        trans(0, 3) = cos_offset * cnt_pt_x + theta_sin * cnt_pt_y;
        trans(1, 3) = cos_offset * cnt_pt_y - theta_sin * cnt_pt_x;

        // Rotate the point cloud
        PointCloudPtr rotated_map(new PointCloud);
        pcl::transformPointCloud(*cloud_map, *rotated_map, trans);

        // Create descriptor and push back
        desc = makeGridDesc(rotated_map, cnt_pt, rows, cols, length, -100, 100);
        descs.push_back(desc);
    }    
}


std::string DIR_MAP = "./";

int main(int argc, char** argv) {
    ros::init(argc, argv, "collect_database");
    ros::NodeHandle nh("~");;

    nh.getParam("DIR_MAP", DIR_MAP);

    std::cout << "==================================================\n";
    std::cout << "==================================================\n";
    std::cout << "||         Begin Offline Collect Database       ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n";

    PointCloudPtr candidate_pts(new PointCloud), pass_map(new PointCloud);

    // candidate pts
    if (pcl::io::loadPCDFile(DIR_MAP + "candidate_pts.pcd", *candidate_pts) == 0) {
        std::cout << "Loaded candidate pts contains: " << candidate_pts->size() << " pts..\n";
    } else {
        ROS_ERROR("Failed to load candidate points from %s", (DIR_MAP + "candidate_pts.pcd").c_str());
    }
    for (auto &pt : candidate_pts->points) {
        pt.z = 0.0f;
    }

    // pass map 
    if (pcl::io::loadPCDFile(DIR_MAP + "pass_map.pcd", *pass_map) == 0) {
        std::cout << "Loaded pass map contains: " << pass_map->size() << " pts..\n";
    } else {
        ROS_ERROR("Failed to load pass map from %s", (DIR_MAP + "pass_map.pcd").c_str());
    }
    // removeOutliersSOR(pass_map, pass_map, 50, 1.5);
    // pcl::io::savePCDFileASCII(DIR_MAP + "pass_map_sor.pcd", *pass_map);

    PointCloudPtr xoy_map(new PointCloud);
    pcl::copyPointCloud(*pass_map, *xoy_map);
    for (auto &pt : xoy_map->points) {
        pt.z = 0.0;
    }
    // pcl::io::savePCDFileASCII(DIR_MAP + "xoy_map.pcd", *xoy_map);

    randomSampleCloud(xoy_map, xoy_map, 500000);
    // pcl::io::savePCDFileASCII(DIR_MAP + "xoy_sam_map.pcd", *xoy_map);

    // collect map database
    float radius = 1.41421 * PARA_ROWS * 0.5 * PARA_LENGTH;
    
    std::cout << "**************************************************\n";
    std::cout << "***   Collecting map database, please don't    ***\n";
    std::cout << "***        shut down... Just a moment!         ***\n";
    std::cout << "**************************************************\n";

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(xoy_map);
    pcl::ExtractIndices<pcl::PointXYZ>::Ptr index_extractor(new pcl::ExtractIndices<pcl::PointXYZ>);
    index_extractor->setInputCloud(xoy_map);
    
    std::vector<Eigen::MatrixXi> descs_total;
    for (int i = 0; i < candidate_pts->size(); ++i) {
        if (!ros::ok()) {
            ROS_WARN("Program interrupted! Stopping processing.");
            break;
        }

        // display progress
        int progress = std::ceil((static_cast<float>(i) / candidate_pts->size()) * 100);
        if (i % 1000 == 0 || i == candidate_pts->size() - 1) {
            std::cout << "### Progress: " << progress << "% (" << i << "/" << candidate_pts->size() << ") ###\n";
        }

        const pcl::PointXYZ& pt = candidate_pts->points[i];
        std::vector<int> indices;
        std::vector<float> distances_sq;
        tree->radiusSearch(pt, radius, indices, distances_sq);

        pcl::IndicesPtr indices_ptr = boost::make_shared<std::vector<int>>(indices);
        index_extractor->setIndices(indices_ptr);
        index_extractor->setNegative(false);
        PointCloudPtr near_map(new PointCloud);
        index_extractor->filter(*near_map);
        
        std::vector<Eigen::MatrixXi> desc_onept;
        simulateOrientationWithinMap(near_map, pt, PARA_ROWS, PARA_COLS, PARA_LENGTH, PARA_THETA_NUM, desc_onept);
        for (int j = 0; j < desc_onept.size(); ++j) {
            descs_total.push_back(desc_onept[j]);
        }
    }
    

    std::cout << "**************************************************\n";
    std::cout << "***      Finding hash keys, please don't       ***\n";
    std::cout << "***        shut down... Just a moment!         ***\n";
    std::cout << "**************************************************\n";
    std::unordered_map<size_t, std::vector<size_t>> key_locations; 
    for (size_t i = 0; i < descs_total.size(); ++i) {
        int progress = std::ceil((static_cast<float>(i) / descs_total.size()) * 100);
        if (i % 50000 == 0 || i == descs_total.size() - 1) {
            std::cout << "### Progress: " << progress << "% (" << i << "/" << descs_total.size() << ") ###\n";
        }

        for (size_t key = 0; key < (PARA_ROWS * PARA_COLS); ++key) {
            if (descs_total[i](0, key) > 0) {
                key_locations[key].push_back(i);
            }
        }
    }

    std::cout << "**************************************************\n";
    std::cout << "***     Saveing offline files, please don't    ***\n";
    std::cout << "***        shut down... Just a moment!         ***\n";
    std::cout << "**************************************************\n";
    for (const auto& pair : key_locations) {
        size_t key = pair.first;
        saveIntegersAsBinary(pair.second, DIR_MAP + "database/" + std::to_string(key) + ".bin");
    }

    return 0;
}




