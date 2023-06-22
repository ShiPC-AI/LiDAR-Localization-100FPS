#include "base/base.h"
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/transforms.h>

void randomSampleCloud(const CloudPtr& cloud_in, 
    CloudPtr& cloud_out, int N) {
    pcl::RandomSample<pcl::PointXYZ> random_sampler;
    random_sampler.setInputCloud(cloud_in);
    random_sampler.setSample(N);
    random_sampler.filter(*cloud_out);
}