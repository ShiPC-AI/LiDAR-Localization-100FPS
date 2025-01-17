#ifndef _LIDAR_100_UTILS_OSC_H_
#define _LIDAR_100_UTILS_OSC_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "for_cloud.hpp"

// mapping
float PARA_GRID_SIZE_MAP = 0.5;
int PARA_MAX_PTS_PER_MAP_GRID = 20;
float PARA_MIN_NOVE = 1.0;
float PARA_GRID_SIZE_GROUND = 1.0;
// float PARA_MIN_HEIGHT = -0.5;
// float PARA_MAX_HEIGHT = 50.0;
float PARA_MIN_Z_MAP = -0.5;
float PARA_MAX_Z_MAP = 50.0;
int PARA_MIN_PTS_PER_GROUND_GRID = 40;
float PARA_VOXEL_LEAF = 0.2;
float PARA_SENSOR_HEIGHT = 1.75;
std::string PARA_DIR_MAP = "./";

// database && relocate
int PARA_ROWS = 40;
int PARA_COLS = 40;
int PARA_THETA_NUM = 60;
float PARA_LENGTH = 1.0;
float PARA_MIN_Z_LOCATE = -0.5;
float PARA_MAX_Z_LOCATE = 50.0;
int PARA_CELL_NUM = PARA_ROWS * PARA_COLS;


Eigen::MatrixXi makeGridDesc(const PointCloudPtr& cloud_in, const pcl::PointXYZ& pt_cnt, 
    const int& rows, const int& cols, const float& length, const float& min_z, const float& max_z) {

    float max_x = pt_cnt.x + rows / 2.0 * length;
    float min_x = pt_cnt.x - rows / 2.0 * length;
    float max_y = pt_cnt.y + cols / 2.0 * length;
    float min_y = pt_cnt.y - cols / 2.0 * length;
    Eigen::MatrixXi desc = Eigen::MatrixXi::Zero(1, rows*cols);

    for (int i = 0; i < cloud_in->size(); ++i) {
        const pcl::PointXYZ &pt = cloud_in->points[i];
        if (pt.z < min_z || pt.z > max_z || pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y) {
            continue;
        }
        int r = (max_x - pt.x) / length;
        int c = (max_y - pt.y) / length; 
        if (r < 0 || r >= rows || c < 0 || c >= cols) {
            continue;
        }
        desc(0, r * cols + c) = 1;
    }
    return desc;
}


#endif