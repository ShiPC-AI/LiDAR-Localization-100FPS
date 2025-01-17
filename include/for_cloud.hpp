#ifndef _LIDAR_100_UTILS_CLOUD_H_
#define _LIDAR_100_UTILS_CLOUD_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <fstream>
#include <iomanip>

// point cloud
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

void randomSampleCloud(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, int N) {
    pcl::RandomSample<pcl::PointXYZ> random_sampler;
    random_sampler.setInputCloud(cloud_in);
    random_sampler.setSample(N);
    random_sampler.filter(*cloud_out);
}

void voxelSampleCloud(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out,
    const float leaf_x, const float leaf_y, const float leaf_z) {
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(cloud_in);
    grid.setLeafSize(leaf_x, leaf_y, leaf_z);
    grid.filter(*cloud_out);
}

void removeOutliersSOR(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, 
    int mean_k, double stddev_mul_thresh) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul_thresh);
    sor.setNegative(false);
    sor.filter(*cloud_out); 
}
#endif