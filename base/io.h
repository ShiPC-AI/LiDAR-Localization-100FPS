#ifndef LL_100FPS_BASE_IO_H_
#define LL_100FPS_BASE_IO_H_

#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "base/utils.hpp"
template <typename T>
bool loadMatrix44(const std::string& filename, Eigen::Matrix<T, 4, 4>& matrix) {
    matrix.setIdentity();
    std::ifstream infile(filename);
    if (!infile.is_open()) {return false;}
    infile >> matrix(0, 0) >> matrix(0, 1) >> matrix(0, 2) >> matrix(0, 3)
           >> matrix(1, 0) >> matrix(1, 1) >> matrix(1, 2) >> matrix(1, 3) 
           >> matrix(2, 0) >> matrix(2, 1) >> matrix(2, 2) >> matrix(2, 3)
           >> matrix(3, 0) >> matrix(3, 1) >> matrix(3, 2) >> matrix(3, 3);
    infile.close();
    return true;
};

template <typename T>
bool saveMatrix44(const std::string& filename, const Eigen::Matrix<T, 4, 4>& matrix) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {return false;}
    outfile << std::setprecision(15)
            << double(matrix(0, 0)) << " " << double(matrix(0, 1)) << " " << double(matrix(0, 2)) << " " << double(matrix(0, 3)) << std::endl
            << double(matrix(1, 0)) << " " << double(matrix(1, 1)) << " " << double(matrix(1, 2)) << " " << double(matrix(1, 3)) << std::endl 
            << double(matrix(2, 0)) << " " << double(matrix(2, 1)) << " " << double(matrix(2, 2)) << " " << double(matrix(2, 3)) << std::endl
            << double(matrix(3, 0)) << " " << double(matrix(3, 1)) << " " << double(matrix(3, 2)) << " " << double(matrix(3, 3));
    outfile.close();
    return true;
};

// binary order: nn_hit_sum, nn_hit_1, nn_hit_2,... nn_hit_119, id_col (each bitset)
bool loadBitsetsColId(const std::string& filename, BitsetVec& bitset_vec,
    int nn_theta, int nn_row, int nn_col);

// binary order: nn_hit_sum, nn_hit_1, nn_hit_2,... nn_hit_119, id_col(each bitset)
bool saveBitsetsColId(const std::string& filename, 
    const BitsetVec& bitset_round, const int nn_theta,
    const int nn_row, const int nn_col);

bool loadMapRings(const std::string& dir_name, BitsetVecVec& map_rings,
    const int nn_pt, const int nn_theta,
    const int nn_row, const int nn_col);
#endif