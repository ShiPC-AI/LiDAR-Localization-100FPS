#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
// #include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/PoseStamped.h>
// #include <iostream>
// #include <fstream>
#include <queue>
#include <mutex>
// #include <Eigen/Dense>
#include <unordered_map>
#include "for_time.hpp"
#include "for_cloud.hpp"
#include "for_desc.hpp"

#include <pcl/filters/extract_indices.h>
#include "offline_mapping/linefit/ground_segmentation.h"
#include "pcl/io/pcd_io.h"
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

std::mutex mBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> _cloud_buf;
std::queue<geometry_msgs::PoseStamped::ConstPtr> _pose_buf;
PointCloudPtr _scan_cloud(new PointCloud);
Eigen::Matrix4f _scan_pose = Eigen::Matrix4f::Identity();

// GridIndex
struct GridIndex {
    int x, y, z;
    bool operator==(const GridIndex &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Use simple bitwise operations to generate hash values
struct GridIndexHash {
    size_t operator()(const GridIndex& grid) const {
        return (std::hash<int>()(grid.x) ^ (std::hash<int>()(grid.y) << 1) ^ (std::hash<int>()(grid.z) << 2));
    }
};
GridIndex calculateGridIndex(const pcl::PointXYZ& point, float voxel_size) {
    return {
        static_cast<int>(std::floor(point.x / voxel_size)),
        static_cast<int>(std::floor(point.y / voxel_size)),
        static_cast<int>(std::floor(point.z / voxel_size))
    };
}

std::unordered_map<GridIndex, std::vector<pcl::PointXYZ>, GridIndexHash> _grid_raw_map;
std::unordered_map<GridIndex, std::vector<pcl::PointXYZ>, GridIndexHash> _grid_pass_map;
std::unordered_map<GridIndex, std::vector<pcl::PointXYZ>, GridIndexHash> _grid_ground;

PointCloudPtr _raw_map(new PointCloud);
PointCloudPtr _pass_map(new PointCloud);
PointCloudPtr _candidate_pts(new PointCloud);

// ground 
PointCloudPtr _ground_scan(new PointCloud);
GroundSegmentationParams _ground_params;
GroundSegmentation _ground_segmenter(_ground_params);

/////////////////////////////////////////////////////////////////////
void recieveCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	mBuf.lock();
	_cloud_buf.push(msg);
	mBuf.unlock();
}
void recievePose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    mBuf.lock();
	_pose_buf.push(msg);
	mBuf.unlock();
}
void removeCloseAndFarPts(const PointCloud &cloud_in, PointCloud &cloud_out, float min_dist, float max_dist) {
    float min_dist2 = min_dist * min_dist;
    float max_dist2 = max_dist * max_dist;
    PointCloud temp_cloud;
    temp_cloud.header = cloud_in.header;
    for (const auto &pt : cloud_in.points) {
        float dist2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (dist2 >= min_dist2 && dist2 <= max_dist2) {
            temp_cloud.points.push_back(pt);
        }
    }
    temp_cloud.width = static_cast<uint32_t>(temp_cloud.points.size());
    temp_cloud.height = 1;
    temp_cloud.is_dense = true;
    cloud_out = std::move(temp_cloud);
}

Eigen::Matrix4f transformMsgToMatrix(const geometry_msgs::PoseStamped::ConstPtr& msg){
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;

    Eigen::Quaternionf quat(qw, qx, qy, qz);
    Eigen::Matrix3f rotation_matrix = quat.toRotationMatrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
    transform_matrix(0, 3) = x;
    transform_matrix(1, 3) = y;
    transform_matrix(2, 3) = z;
    return transform_matrix;
}

void addPassOutliersToGrid(const pcl::PointXYZ& point) {
    GridIndex grid_index = calculateGridIndex(point, PARA_GRID_SIZE_MAP);
    if (_grid_raw_map[grid_index].size() < PARA_MAX_PTS_PER_MAP_GRID) {
        _grid_raw_map[grid_index].push_back(point);    
        _raw_map->points.push_back(point);    
    }
}

void addPassInliersToGrid(const pcl::PointXYZ& point) {
    GridIndex grid_index = calculateGridIndex(point, PARA_GRID_SIZE_MAP);
    if (_grid_raw_map[grid_index].size() < PARA_MAX_PTS_PER_MAP_GRID) { // raw map
        _grid_raw_map[grid_index].push_back(point);    
        _raw_map->points.push_back(point);    
    }
    if (_grid_pass_map[grid_index].size() < PARA_MAX_PTS_PER_MAP_GRID) { //pass map
        _grid_pass_map[grid_index].push_back(point);    
        _pass_map->points.push_back(point);   
    }
}

void addGroundToGrid(const pcl::PointXYZ& point) {
    GridIndex grid_index = calculateGridIndex(point, PARA_GRID_SIZE_GROUND);
    if (_grid_ground[grid_index].size() < PARA_MIN_PTS_PER_GROUND_GRID) {
        _grid_ground[grid_index].push_back(point);
        if (_grid_ground[grid_index].size() == PARA_MIN_PTS_PER_GROUND_GRID) {
            pcl::PointXYZ pt;
            pt.x = (grid_index.x + 0.5) * PARA_GRID_SIZE_GROUND;
            pt.y = (grid_index.y + 0.5) * PARA_GRID_SIZE_GROUND;
            pt.z = point.z;
            _candidate_pts->points.push_back(std::move(pt));
        }
    } 
}

void passThrough(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_inlier, PointCloudPtr& cloud_outlier,
    const float min_x, const float max_x, const float min_y, const float max_y, const float min_z, const float max_z){
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
    for (int i = 0; i < cloud_in->size(); ++i) {
        const pcl::PointXYZ& pt = cloud_in->points[i];
        if (pt.z < min_z || pt.z > max_z || pt.y < min_y || pt.y > max_y 
            || pt.x < min_x || pt.x > max_x) {
            continue;
        }
        inliers_ptr->indices.push_back(i);
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers_ptr);
    extract.setNegative(false);
    extract.filter(*cloud_inlier);
    extract.setNegative(true);
    extract.filter(*cloud_outlier);
    return;
}

void segmentGroundFromScan(const PointCloudPtr& cloud_in) {
    _ground_scan.reset(new PointCloud);
    std::vector<int> labels;
    _ground_segmenter.segment(*cloud_in, &labels);
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] == 1) {
            _ground_scan->push_back(cloud_in->points[i]);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh("~");;

    nh.getParam("PARA_GRID_SIZE_MAP", PARA_GRID_SIZE_MAP);
    nh.getParam("PARA_MIN_NOVE", PARA_MIN_NOVE);
    nh.getParam("PARA_MAX_PTS_PER_MAP_GRID", PARA_MAX_PTS_PER_MAP_GRID);
    nh.getParam("PARA_MIN_PTS_PER_GROUND_GRID", PARA_MIN_PTS_PER_GROUND_GRID);
    nh.getParam("PARA_MIN_Z_MAP", PARA_MIN_Z_MAP);
    nh.getParam("PARA_MAX_Z_MAP", PARA_MAX_Z_MAP);
    nh.getParam("PARA_GRID_SIZE_GROUND", PARA_GRID_SIZE_GROUND);
    nh.getParam("PARA_DIR_MAP", PARA_DIR_MAP);
    nh.getParam("PARA_VOXEL_LEAF", PARA_VOXEL_LEAF);
    nh.getParam("PARA_SENSOR_HEIGHT", PARA_SENSOR_HEIGHT);
    _ground_params.setSensorHeight(PARA_SENSOR_HEIGHT);

    // subscriber
    ros::Subscriber sub_scan = nh.subscribe("/velodyne_points", 1000, recieveCloud);
    ros::Subscriber sub_pose = nh.subscribe("/lidar_pose", 1000, recievePose);

    // publisher
    ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map_raw_cloud", 1);
    ros::Publisher pub_pass_map = nh.advertise<sensor_msgs::PointCloud2>("/map_pass_cloud", 1);
    ros::Publisher pub_candidate = nh.advertise<sensor_msgs::PointCloud2>("/candidate_pts", 1);

    std::cout << "==================================================\n";
    std::cout << "||                                              ||\n";
    std::cout << "||             Begin Offline Mapping            ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "||       Waiting for LiDRA scans to arrive      ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n";

    int keyframe_count = 0;
    Eigen::Vector3f curr_xyz = Eigen::Vector3f::Zero();
    Eigen::Vector3f last_xyz = Eigen::Vector3f::Zero();

    ros::Rate loop_rate(20); 
    ros::Time last_publish_time = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce(); 
        
        // trigger saving map point cloud
        if ((ros::Time::now() - last_publish_time).toSec() > 6 && _cloud_buf.empty() && _pose_buf.empty()) {
            if (!_pass_map->empty() && !_candidate_pts->empty() && !_raw_map->empty()) {
                ROS_WARN("No message publish for 6 seconds. Saving map...");
                std::cout << "Candidates contains " << static_cast<int>(_candidate_pts->size()) << " pts...\n";
                std::cout << "Pass map contains " << static_cast<int>(_pass_map->size()) << " pts..\n";
                std::cout << "Raw map contains " << static_cast<int>(_raw_map->size()) << " pts..\n";

                _candidate_pts->width = static_cast<uint32_t>(_candidate_pts->size());
                _candidate_pts->height = 1;
                _candidate_pts->is_dense = true;
                _pass_map->width = static_cast<uint32_t>(_pass_map->size());
                _pass_map->height = 1;
                _pass_map->is_dense = true;
                _raw_map->width = static_cast<uint32_t>(_raw_map->size());
                _raw_map->height = 1;
                _raw_map->is_dense = true;
                
                // candidates
                removeOutliersSOR(_candidate_pts, _candidate_pts, 50, 1.0);
                if (pcl::io::savePCDFileASCII(PARA_DIR_MAP + "candidate_pts.pcd", *_candidate_pts) == 0) {
                    std::cout << "Successfully saved candidates, " << _candidate_pts->size()<< " pts...\n";
                } else {
                    ROS_ERROR("Failed to save candidates in %s", (PARA_DIR_MAP + "candidate_pts.pcd").c_str());
                }
                
                // pass_map
                voxelSampleCloud(_pass_map, _pass_map, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF);
                if ( pcl::io::savePCDFileASCII(PARA_DIR_MAP + "pass_map.pcd", *_pass_map) == 0) {
                    std::cout << "Successfully saved pass map, " << _pass_map->size()<< " pts...\n";
                } else {
                    ROS_ERROR("Failed to save pass map in %s", (PARA_DIR_MAP + "pass_map.pcd").c_str());
                }

                // raw_map
                voxelSampleCloud(_raw_map, _raw_map, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF);
                if ( pcl::io::savePCDFileASCII(PARA_DIR_MAP + "raw_map.pcd", *_raw_map) == 0) {
                    std::cout << "Successfully saved raw map, " << _raw_map->size()<< " pts...\n";
                } else {
                    ROS_ERROR("Failed to save raw map in %s", (PARA_DIR_MAP + "raw_map.pcd").c_str());
                }

                _raw_map->clear();
                _pass_map->clear();
                _candidate_pts->clear();
            }
        }

        if (_cloud_buf.empty() || _pose_buf.empty()) {
            continue;
        }
        
        double time_cloud = _cloud_buf.front()->header.stamp.toSec();
        double time_pose = _pose_buf.front()->header.stamp.toSec();
        if (time_cloud != time_pose) {
            std::printf("unsync messeage!");
            ROS_BREAK();
        } 
        
        mBuf.lock();
        _scan_cloud->clear();
        pcl::fromROSMsg(*_cloud_buf.front(), *_scan_cloud);
        _cloud_buf.pop();

        geometry_msgs::PoseStamped::ConstPtr pose_msg = _pose_buf.front();
        _scan_pose = transformMsgToMatrix(pose_msg);
        _pose_buf.pop();
        mBuf.unlock();
        
        // generate new keyframe each PARA_MIN_NOVE
        curr_xyz = _scan_pose.block<3, 1>(0, 3);
        if ((curr_xyz - last_xyz).norm() < PARA_MIN_NOVE) {
            last_publish_time = ros::Time::now(); // time reset
            continue;
        } else {
            ++keyframe_count;
            last_xyz = curr_xyz;
        }
        
        TicToc tic_total;

        // remove near and far pts
        removeCloseAndFarPts(*_scan_cloud, *_scan_cloud, 5.0, 50.0);

        // segment ground using linefit method
        segmentGroundFromScan(_scan_cloud);
        pcl::transformPointCloud(*_ground_scan, *_ground_scan, _scan_pose);
        for (const auto& pt : _ground_scan->points) {
            addGroundToGrid(pt);
        }
        
        // voxel and pass through scan
        voxelSampleCloud(_scan_cloud, _scan_cloud, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF);
        PointCloudPtr _scan_pass_in(new PointCloud),_scan_pass_out(new PointCloud);
        passThrough(_scan_cloud, _scan_pass_in, _scan_pass_out, -80, 80, -80, 80, PARA_MIN_Z_MAP, PARA_MAX_Z_MAP);

        // update map points
        pcl::transformPointCloud(*_scan_pass_in, *_scan_pass_in, _scan_pose);
        for (const auto& pt : _scan_pass_in->points) {
            addPassInliersToGrid(pt);
        }
        pcl::transformPointCloud(*_scan_pass_out, *_scan_pass_out, _scan_pose);
        for (const auto& pt : _scan_pass_out->points) {
            addPassOutliersToGrid(pt);
        }

        std::cout << keyframe_count << "th keyframe, takes: "<< tic_total.toc() << "ms\n";
        std::cout << "Raw map:  " << _raw_map->size() << "\n";
        std::cout << "Pass map: " << _pass_map->size() << "\n";

        // message
        sensor_msgs::PointCloud2 msg_candidate;
        pcl::toROSMsg(*_candidate_pts, msg_candidate);
        msg_candidate.header.stamp = ros::Time::now();
        msg_candidate.header.frame_id = "map";
        pub_candidate.publish(msg_candidate);  

        sensor_msgs::PointCloud2 msg_raw_map;
        pcl::toROSMsg(*_raw_map, msg_raw_map);
        msg_raw_map.header.stamp = ros::Time::now();
        msg_raw_map.header.frame_id = "map";  
        pub_map.publish(msg_raw_map);        

        sensor_msgs::PointCloud2 msg_pass_map;
        pcl::toROSMsg(*_pass_map, msg_pass_map);
        msg_pass_map.header.stamp = ros::Time::now();
        msg_pass_map.header.frame_id = "map"; 
        pub_pass_map.publish(msg_pass_map);       
           
        last_publish_time = ros::Time::now();// time reset
        loop_rate.sleep(); 
    }
    return 0;
}
