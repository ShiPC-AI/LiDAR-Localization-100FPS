#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

#include "for_time.hpp"
#include "for_cloud.hpp"
#include "for_desc.hpp"
#include "for_io.hpp"
#include "pcl/io/pcd_io.h"
#include <pcl/common/transforms.h>
#include <algorithm> // for std::max_element
#include <omp.h>

PointCloudPtr _candidate_pts(new PointCloud);
pcl::PointXYZ _cnt_pt(0.0f, 0.0f, 0.0f);
float _theta_reso = M_PI * 2.0 / (float) PARA_THETA_NUM;
int  _desc_total_num = 0;

Eigen::Matrix3f _rot_relocate = Eigen::Matrix3f::Identity();
Eigen::Matrix4f _pose_relocate = Eigen::Matrix4f::Identity();
pcl::PointXYZ _xy_result = pcl::PointXYZ(0.0f, 0.0f, 0.0f);
float _theta_result = 0;
int _theta_index = 0;
int _pt_index = 0;
int _max_vote = 0;

std::mutex mBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> _cloud_buf;
pcl::PointCloud<pcl::PointXYZ>::Ptr _scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::unordered_map<size_t, std::vector<size_t>>  _key_locations;

void recieveCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
	mBuf.lock();
	_cloud_buf.push(cloud_msg);
	mBuf.unlock();
}


void findMaxVotes(const Eigen::MatrixXi& query) {
    _max_vote = 0;
    std::vector<int> votes(_desc_total_num, 0); 

    omp_set_num_threads(8);
    #pragma omp parallel for
    for (size_t key = 0; key < PARA_CELL_NUM; ++key) {
        if (query(key) && _key_locations.count(key)) { 
            for (size_t index : _key_locations.at(key)) {
                #pragma omp atomic
                ++votes[index];
            }
        }
    } 

    auto max_iter = std::max_element(votes.begin(), votes.end());
    const int max_index = std::distance(votes.begin(),  max_iter);

    _pt_index = max_index / PARA_THETA_NUM;
    _theta_index = max_index % PARA_THETA_NUM;
    _max_vote = *max_iter;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "relocate");
    ros::NodeHandle nh("~");;

    nh.getParam("PARA_DIR_MAP", PARA_DIR_MAP);
    nh.getParam("PARA_MIN_Z_LOCATE", PARA_MIN_Z_LOCATE);
    nh.getParam("PARA_MAX_Z_LOCATE", PARA_MAX_Z_LOCATE);

    ros::Publisher pub_aligned_scan = nh.advertise<sensor_msgs::PointCloud2>("/aligned_scan", 1000);
    ros::Subscriber sub_scan = nh.subscribe("/velodyne_points", 1000, recieveCloud);

    std::cout << "==================================================\n";
    std::cout << "||                                              ||\n";
    std::cout << "||           Loading Map Database..             ||\n";
    std::cout << "||     Please do not turn off the terminal.     ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n";

    // candidate pts
    if (pcl::io::loadPCDFile(PARA_DIR_MAP + "candidate_pts.pcd", *_candidate_pts) == 0) {
        std::cout << "Loaded candidate pts contains:" << _candidate_pts->size() << " points\n";
    } else {
        ROS_ERROR("Failed to load [[ Candidate Pts ]] from %s", (PARA_DIR_MAP + "candidate_pts.pcd").c_str());
    }
    _desc_total_num = _candidate_pts->size() * PARA_THETA_NUM;

    TicToc timer_database;
    for (size_t key = 0; key < PARA_CELL_NUM; ++key) {
        if (key % 400 == 0) {
            int progress = std::ceil(((float)key / PARA_CELL_NUM) * 100);
            std::cout << "### Progress: " << progress << "% (" << key << "/" << PARA_CELL_NUM << ") ###\n";
        }

        std::vector<size_t> numbers;
        readIntegersFromBinary(PARA_DIR_MAP + "database/" + std::to_string(key) + ".bin", numbers);
        _key_locations[key].assign(numbers.begin(), numbers.end());
    }
    std::cout << "Loading database takes: " << timer_database.toc() << "ms\n\n";
    

    std::cout << "==================================================\n";
    std::cout << "||                                              ||\n";
    std::cout << "||          Begin Relocate Wihtin Map           ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "||    LiDAR scans can be played at any time,    ||\n";
    std::cout << "||         without requiring sequential.        ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n\n";

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce(); 
        if (_cloud_buf.empty()) {
            continue;
        }

        _scan_cloud->clear();
        pcl::fromROSMsg(*_cloud_buf.front(), *_scan_cloud);
        _cloud_buf.pop();
        TicToc timer_locate;

        // TicToc timer_voxel;
        voxelSampleCloud(_scan_cloud, _scan_cloud, 0.2, 0.2, 0.2);
        // std::cout << "Voxel takes: " << timer_voxel.toc() << "ms\n";

        // TicToc timer_desc;
        Eigen::MatrixXi scan_desc = makeGridDesc(_scan_cloud, _cnt_pt, PARA_ROWS, PARA_COLS, PARA_LENGTH, PARA_MIN_Z_LOCATE, PARA_MAX_Z_LOCATE); 
        // std::cout << "Desc takes: " << timer_desc.toc() << "ms\n";

        findMaxVotes(scan_desc);
        std::cout << "Relocated takes: " << timer_locate.toc() << "ms\n";

        // get pose
        _xy_result = _candidate_pts->at(_pt_index);
        _theta_result = _theta_index * _theta_reso;
        
        _rot_relocate = Eigen::Matrix3f::Identity();
        _rot_relocate(0, 0) =  std::cos(_theta_result); 
        _rot_relocate(0, 1) = -std::sin(_theta_result); 
        _rot_relocate(1, 0) =  std::sin(_theta_result); 
        _rot_relocate(1, 1) =  std::cos(_theta_result);

        _pose_relocate = Eigen::Matrix4f::Identity();
        _pose_relocate.block<3, 3>(0, 0) = _rot_relocate.inverse();
        _pose_relocate(0, 3) = _xy_result.x; 
        _pose_relocate(1, 3) = _xy_result.y;

        pcl::transformPointCloud(*_scan_cloud, *_scan_cloud, _pose_relocate);

        sensor_msgs::PointCloud2 aligned_scan_msg;
        pcl::toROSMsg(*_scan_cloud, aligned_scan_msg);
        aligned_scan_msg.header.stamp = ros::Time::now();
        aligned_scan_msg.header.frame_id = "map";
        pub_aligned_scan.publish(aligned_scan_msg);
        
        loop_rate.sleep();
    }

    return 0;
}
