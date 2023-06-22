#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h> 
#include "extractor/square.h"
#include "base/base.h"
#include "base/io.h"

void makeRoundSquare(pcl::ExtractIndices<pcl::PointXYZ>::Ptr& idx_ext,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
    BitsetVec& squares, pcl::PointXYZ& pt_ori, 
    int nn_theta, int nn_row, int nn_col, double d_cell) {

    float dd = std::sqrt(2) * nn_row * 0.5 * d_cell;
    CloudPtr cloud_range(new PointCloud);
    std::vector<int> indice_vec;
    std::vector<float> dists_vec;
    tree->radiusSearch(pt_ori, dd, indice_vec, dists_vec);
    pcl::IndicesPtr indices_ptr = std::make_shared<std::vector<int>>(indice_vec);
    idx_ext->setIndices(indices_ptr);
    idx_ext->setNegative(false);
    idx_ext->filter(*cloud_range);

    squares.clear();
    squares.resize(nn_theta);
    if (cloud_range->empty()) {
        for (int i = 0; i < squares.size(); ++i) {
            squares[i].reset();
        }
        return;
    }

    SquareExtractor square_extractor;
    square_extractor.setCellRows(nn_row);
    square_extractor.setCellCols(nn_col);
    square_extractor.setCellLength(d_cell);
    square_extractor.setPassParas(z_min_, z_max_);
    square_extractor.setOrigin(pt_ori);
    square_extractor.setSampleNum(nn_sample_);

    float step_theta = PI_2 / nn_theta;
    for (int i = 0; i < nn_theta; ++i) {
        Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
        double theta = i * step_theta;
        double theta_cos = std::cos(theta);
        double theta_sin = std::sin(theta);        
        trans(0, 0) = theta_cos; trans(0, 1) = -theta_sin; 
        trans(1, 0) = theta_sin; trans(1, 1) =  theta_cos;
        trans(0, 3) = (1 - theta_cos) * pt_ori.x + theta_sin * pt_ori.y;
        trans(1, 3) = (1 - theta_cos) * pt_ori.y - theta_sin * pt_ori.x;
        CloudPtr cloud_range_trans(new PointCloud);
        pcl::transformPointCloud(*cloud_range, *cloud_range_trans, trans);

        Bitset square_theta;
        square_extractor.setInputCloud(cloud_range_trans);
        square_extractor.segment(square_theta);
        squares[i] = square_theta;
    }    
}

int main(int argc, char** argv) {
    // std::string dir_map = "../data/corridor/map/";
    std::string dir_map = "../data/lobby/map/";
    std::string path_map_pro = dir_map + "map_sam_pro.pcd";
    std::string path_map_candidate = dir_map + "map_candidate.pcd"; // 0.3m
    std::string name_map_square = str_nn_rows_ + "_" + str_nn_cols_ + "_" + str_nn_theta_ + "_" + str_d_cell_ ;// 40_40_120_1

    // Assume that: 
    // (1) map candidate point cloud has been generated
    // (2) map projected point cloud has been generated.
    // *************************************************
    // *************************************************
    // *************************************************

    // Load the sampled and projected map 
    CloudPtr cloud_map_project(new PointCloud);
    pcl::io::loadPCDFile(path_map_pro, *cloud_map_project);
    std::cout << "Sampled and prjected map size: " << cloud_map_project->size() << ".\n";

    // Load the map candidate point cloud
    CloudPtr cloud_map_candidate(new PointCloud);
    pcl::io::loadPCDFile(path_map_candidate, *cloud_map_candidate);
    std::cout << "Map candidate points size: " << cloud_map_candidate->size() << ".\n";

    // Build Kdtree && build indices extractor
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_map_project);
    pcl::ExtractIndices<pcl::PointXYZ>::Ptr idx_ext(new pcl::ExtractIndices<pcl::PointXYZ>);
    idx_ext->setInputCloud(cloud_map_project);

    // Save square templates
    // std::cout << "Parameter name: " << name_map_square << "\n";
    std::cout << "====Processing: save map descriptor, don't shutdown====\n";
    for (int i_pt = 0; i_pt < cloud_map_candidate->size(); ++i_pt) {
        if (i_pt % 400 == 0) {
            std::cout << i_pt << "th pt\n";
        }
        BitsetVec squares;
        pcl::PointXYZ& pt = cloud_map_candidate->points[i_pt];
        makeRoundSquare(idx_ext, tree, squares, pt, nn_theta_, nn_rows_, nn_cols_, d_cell_);
        
        std::string path_squares = dir_map + "squares/" + name_map_square + "/" + std::to_string(i_pt) + ".dat";
        if (!saveBitsetsColId(path_squares, squares, nn_theta_, nn_rows_, nn_cols_)) {
            std::cout << "Save squares failed, please check path!!!\n";
            return -1;
        }
    }
}