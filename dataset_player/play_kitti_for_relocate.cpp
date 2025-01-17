
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <signal.h> 
#include <algorithm> // std::shuffle
#include <random>    // std::mt19937, std::random_device
#include <queue>

bool g_shutdown_requested = false;

// Ctrl+C信号处理器
void sigintHandler(int sig) {
    ROS_INFO("Ctrl-C detected, shutting down...");
    g_shutdown_requested = true;
}

void loadBinToCloudXYZ(const std::string& bin_path, int index, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "play_kitti_for_relocate");
    ros::NodeHandle nh("~");

    std::string dataset_folder;
    std::string seq_number;
    int publish_delay;

    nh.getParam("dataset_folder", dataset_folder);
    nh.getParam("seq_number", seq_number);
    nh.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    std::cout << "======== Reading KITTI " << seq_number << " from: " << dataset_folder << " ========\n";

    ros::Publisher pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 3000);
    std::ifstream timestamp_file(dataset_folder + seq_number + "/times.txt", std::ifstream::in);
    std::cout << dataset_folder + seq_number + "/times.txt" << "\n";

    ros::Duration(1.0).sleep(); 
    signal(SIGINT, sigintHandler);

    std::string line;
    std::vector<float> times;
    while(1) {
        if (std::getline(timestamp_file, line)) {
            float timestamp = stof(line);        
            times.push_back(timestamp);
        } else {
            break;
        }
    }
    std::cout << "Total times: " << times.size() << "\n";

    std::vector<int> indexes(times.size());  
    std::iota(indexes.begin(), indexes.end(), 0);  // fill 0-N
    
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indexes.begin(), indexes.end(), g);  // shuffle
    
    std::queue<int> index_queue;
    for (int i = 0; i < indexes.size(); ++i) {
        index_queue.push(indexes[i]);
    }

    ros::Rate r(10.0 / publish_delay);
    while (ros::ok() && !g_shutdown_requested) {
        if (index_queue.empty()) {
            break;
        }
        int data_id = index_queue.front();
        index_queue.pop();
        std::cout << data_id << "th pcd\n"; 
        std::string bin_path = dataset_folder + seq_number + "/velodyne/";
        pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        loadBinToCloudXYZ(bin_path, data_id, laser_cloud);

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(*laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(times[data_id]);
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);
        r.sleep();
    }
    
    return 0;
}