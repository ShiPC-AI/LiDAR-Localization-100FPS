#include "extractor/square.h"
#include "localizer/square.h"
#include "base/base.h"
#include "base/io.h"

int main(int argc, char** argv) {
        
    // std::string dir_map = "../data/corridor/map/";
    std::string dir_map = "../data/lobby/map/";
    std::string dir_scan = "../data/lobby/scan/";

    std::string name_map_square = str_nn_rows_ + "_" + str_nn_cols_ + "_" + str_nn_theta_ + "_" + str_d_cell_;
    std::string dir_map_square = dir_map + "squares/" + name_map_square + "/";
    std::string path_map_candidate = dir_map + "map_candidate.pcd"; 

    // load map candidate
    CloudPtr cloud_map_candidate(new PointCloud);
    pcl::io::loadPCDFile(path_map_candidate, *cloud_map_candidate);
    int nn_candidate = (int) cloud_map_candidate->size();
    std::cout << "Map candidate points size: " << cloud_map_candidate->size() << ".\n";

    // load map squares
    std::cout << "====Processing: loading map descriptor, don't shutdown====\n";

    BitsetVecVec map_squares(nn_candidate);
    std::clock_t tt_load_1 = std::clock();
    if (!loadMapRings(dir_map_square, map_squares, nn_candidate, nn_theta_, nn_rows_, nn_cols_)) {
        return -1;
    }   
    std::clock_t tt_load_2 = std::clock();
    double tt_load = double(tt_load_2 - tt_load_1) * MS_SEC;
    // std::cout << "Map descriptors--pt size: " << map_squares.size() << ".\n";
    // std::cout << "Map descriptors--theta size: " << map_squares[0].size() << ".\n";
    std::cout << "Time of loading map descriptors: " << tt_load << "ms.\n";    

    LocalizerSquare localizer;
    localizer.setThetaNum(nn_theta_);
    localizer.setTreeNum(nn_tree_);
    localizer.setNodeNum(nn_node_);
    localizer.setTopCandidateNum(nn_top_);
    localizer.setLocateMode(flat_locate_);
    localizer.loadMapCandidatePts(cloud_map_candidate);

    std::clock_t tt_tree_1 = std::clock();
    localizer.loadMapDescriptors(map_squares);
    std::clock_t tt_tree_2 = std::clock();
    double tt_tree = double(tt_tree_2 - tt_tree_1) * MS_SEC;
    std::cout << "Time of building tree: " << tt_tree << "ms.\n";    

    std::cout << "====Processing: global localization====\n";
    for (int i = 100; i < 105; ++i) {
        CloudPtr cloud_scan(new PointCloud);
        pcl::io::loadPCDFile(dir_scan + std::to_string(i) + ".pcd", *cloud_scan);

        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        if (!loadMatrix44(dir_scan + std::to_string(i) + ".pose", pose)) {
            std::cout << "Open pose faild, please check path!!!"; 
            return -1;
        }
        
        // Extract descriptor
        std::clock_t tt_desc_1 = std::clock();
        Bitset square_scan;
        pcl::PointXYZ pt_ori(0.0, 0.0, 0.0);
        SquareExtractor square_extractor;
        square_extractor.setCellRows(nn_rows_);
        square_extractor.setCellCols(nn_cols_);
        square_extractor.setCellLength(d_cell_);
        square_extractor.setOrigin(pt_ori);
        square_extractor.setPassParas(z_min_, z_max_);
        square_extractor.setSampleNum(nn_sample_);
        // std::clock_t tt_sam_1 = std::clock();
        square_extractor.setInputCloud(cloud_scan);
        // std::clock_t tt_sam_2 = std::clock();
        // double tt_sam = double(tt_sam_2 - tt_sam_1) * MS_SEC;

        square_extractor.segment(square_scan);
        std::clock_t tt_desc_2 = std::clock();
        double tt_desc = double(tt_desc_2 - tt_desc_1) * MS_SEC;
        
        // Compute vehicle poses
        Mat44dVec pose_vec;
        localizer.loadScanDescripor(square_scan);
        std::clock_t tt_locate_1 = std::clock();
        localizer.startLocate(pose_vec); 
        std::clock_t tt_locate_2 = std::clock();
        double tt_locate = double(tt_locate_2 - tt_locate_1) * MS_SEC;
        
        // Compute localization error
        Eigen::Matrix4d pose_init = pose_vec[0];
        // float dd = (pose_init - pose).block<3, 1>(0, 3).norm();
        float dd_xy = (pose_init - pose).block<2, 1>(0, 3).norm();
        Eigen::Matrix3d rot_1 = pose.block<3, 3>(0, 0);
        Eigen::Matrix3d rot_init = pose_init.block<3, 3>(0, 0);
        float hh = 180.0 / M_PI * std::acos(((rot_1.inverse() * rot_init).trace() - 1) * 0.5);

        std::cout << i << "th scan, " << "dd: " << dd_xy << "m, " 
                                      << "hh: " << hh << "deg, " 
                                      << "tt-desc: " << tt_desc << "ms, "
                                      << "tt-loc: " << tt_locate << "ms.\n";
    }
    return 1;
}
