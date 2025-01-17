#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "for_cloud.hpp"
#include "for_desc.hpp"
#include "for_time.hpp"
PointCloudPtr _raw_map(new PointCloud);

int main(int argc, char** argv) {
    ros::init(argc, argv, "load_map");
    ros::NodeHandle nh("~");;

    nh.getParam("PARA_DIR_MAP", PARA_DIR_MAP);
    ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/raw_map", 1);
    
    TicToc timer_map;
    if (pcl::io::loadPCDFile(PARA_DIR_MAP + "raw_map.pcd", *_raw_map)) {
        ROS_ERROR("Failed to load [[ Raw Map ]] from %s", (PARA_DIR_MAP + "raw_map.pcd").c_str());
        return -1;
    } 
    // removeOutliersSOR(_raw_map, _raw_map, 50, 2.0);
    std::cout << "==================================================\n";
    std::cout << "||                                              ||\n";
    std::cout << "||       Point Cloud Map Loaded Successfully.   ||\n";
    std::cout << "||            Ready for LiDAR operation         ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n";
    std::cout << "Loaded raw map contains: " << _raw_map->size() << " pts..\n";
    std::cout << "Loading map takes: " << timer_map.toc() << "ms\n\n";

    ros::Rate loop_rate(0.01);
    while (ros::ok()) {
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*_raw_map, map_msg);
        map_msg.header.stamp = ros::Time::now();
        map_msg.header.frame_id = "map";
        pub_map.publish(map_msg);
        loop_rate.sleep();
    }

    return 0;
}
