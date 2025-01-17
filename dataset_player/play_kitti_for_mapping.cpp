
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
#include <geometry_msgs/TransformStamped.h>

bool g_shutdown_requested = false;

void sigintHandler(int sig) {
    ROS_INFO("Ctrl-C detected, shutting down...");
    g_shutdown_requested = true;
}

void loadPoseToMatrix(const std::string& filename, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poses) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));
    }

    std::vector<std::vector<double>> values;
    std::string line;
    while (std::getline(ifs, line)) {
        std::istringstream ss(line);
        values.emplace_back(std::istream_iterator<double>{ss}, std::istream_iterator<double>{}); // C++ >=17
    }

    if (values.empty() || values[0].empty()) {
        std::exit((std::cout << "Values data is empty or malformed.", EXIT_FAILURE));
    }

    for (int i = 0; i < (int)values.size(); ++i) {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        for (int j = 0; j < (int)values[0].size(); ++j) {
            int row = j / 4;
            int col = j % 4;
            pose(row, col) = values[i][j];
        }
        poses.push_back(pose);
    } 

    ifs.close();
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
    ros::init(argc, argv, "play_kitti_for_mapping");
    ros::NodeHandle nh("~");

    std::string dataset_folder;
    std::string seq_number;
    int publish_delay;

    nh.getParam("dataset_folder", dataset_folder);
    nh.getParam("seq_number", seq_number);
    nh.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    std::cout << "======== Reading KITTI " << seq_number << " from: " << dataset_folder << " ========\n";
 
    // pose file 
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses;
    std::string path_pose = dataset_folder + seq_number + "/lidar_pose.txt";
    loadPoseToMatrix(path_pose, poses);
    std::cout << "Pose total: " << poses.size() << "\n";

    // publisher
    ros::Publisher pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1000);
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/lidar_pose", 1000);
    geometry_msgs::PoseStamped pose_msg;

    // add a delay to wait subscriber
    ros::Duration(1.0).sleep(); 

    // Ctrl+C signal
    signal(SIGINT, sigintHandler);

    std::string line;
    ros::Rate r(10.0 / publish_delay);
    int line_count = 0;
    std::ifstream time_file(dataset_folder + seq_number + "/times.txt", std::ifstream::in);

    while (ros::ok() && !g_shutdown_requested) {
        if (std::getline(time_file, line)) {
            // std::cout << "playing: " << line_count << "th pcd" << "\n";
            
            // times && cloud
            float timestamp = stof(line);        
            std::string bin_path = dataset_folder + seq_number + "/velodyne/";
            pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            loadBinToCloudXYZ(bin_path, line_count, laser_cloud);

            // transform
            Eigen::Matrix4f pose_matrix = poses[line_count];
            Eigen::Matrix3f rotation = pose_matrix.block<3, 3>(0, 0);
            Eigen::Quaternionf quaternion(rotation);
            
            pose_msg.pose.position.x = pose_matrix(0, 3);
            pose_msg.pose.position.y = pose_matrix(1, 3);
            pose_msg.pose.position.z = pose_matrix(2, 3);
            pose_msg.pose.orientation.x = quaternion.x();
            pose_msg.pose.orientation.y = quaternion.y();
            pose_msg.pose.orientation.z = quaternion.z();
            pose_msg.pose.orientation.w = quaternion.w();
            pose_msg.header.stamp = ros::Time().fromSec(timestamp);
            pose_msg.header.frame_id = "/map";  

            // publish
            pub_pose.publish(pose_msg);

            sensor_msgs::PointCloud2 laser_cloud_msg;
            pcl::toROSMsg(*laser_cloud, laser_cloud_msg);
            laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
            laser_cloud_msg.header.frame_id = "/camera_init";
            pub_laser_cloud.publish(laser_cloud_msg);

            ++line_count;
            r.sleep();
        } else {
            break; // exit
        }
    }
    return 0;
}