#ifndef LL_100FPS_BASE_UTILS_H_
#define LL_100FPS_BASE_UTILS_H_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <fstream>
#include <iostream>
#include <numeric>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <bitset>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

// const double PI_A = 180.0 / M_PI;
// const double PI_R = M_PI / 180.0;
const double PI_2 = M_PI * 2.0; 
const double MS_SEC = (double)1000 / CLOCKS_PER_SEC;

// parameter of square template
static const int nn_cell_ = 1600; // cell number of squasre
static const int nn_rows_ = 40; // row number of square 
static const int nn_cols_ = 40; // col number of square
static const int nn_theta_ = 120; // theta number (resultion = 360.0 / nn_theta_)
static const float d_cell_ = 1.0; // length of cell
// static const float d_map_ = 0.4; // resolution of map candidate
static const float z_min_ = -0.1; // min height -0.5
static const float z_max_ = 2.2; // max height 1.1
static const int nn_sample_ = 8000; // sampling number 

// parameter of localizer
static const int nn_tree_ = 1; // number of tree
static const int nn_node_ = 2048; // number of node, -1:search all nodes
static const int nn_top_ = 1; // number of top candidate
static const bool flat_locate_ = true; // localization flag, true-fast locate, fasle-brute force 

//string format of parameter
static std::string str_d_cell_ = "1"; // cell length
// static std::string str_d_map_ = "0.4"; // map candidate resolution
static std::string str_nn_cell_ = std::to_string(nn_cell_); // cell number
static std::string str_nn_rows_ = std::to_string(nn_rows_); // row number
static std::string str_nn_cols_ = std::to_string(nn_cols_); // column number
static std::string str_nn_theta_ = std::to_string(nn_theta_); // theta number

typedef std::bitset<nn_cell_> Bitset;
typedef std::vector<Bitset> BitsetVec;
typedef std::vector<BitsetVec> BitsetVecVec;
typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Mat44dVec;
typedef std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>> Vec4iVec;
#endif /*LL_100FPS_BASE_UTILS_H_*/