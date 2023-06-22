#ifndef LL_100FPS_LOCALIZER_SQUARE_H_
#define LL_100FPS_LOCALIZER_SQUARE_H_
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/flann.hpp>
#include "base/utils.hpp"
class LocalizerSquare {
public:
    LocalizerSquare(){};
    ~LocalizerSquare(){};
    void loadScanDescripor(Bitset &desc_in);
    void loadMapCandidatePts(CloudPtr& cloud_in);
    void loadMapDescriptors(BitsetVecVec &descs_in);
    
    void startLocate(Mat44dVec& pose_vec);

    void setThetaNum(int nn);
    void setTreeNum(int nn);
    void setNodeNum(int nn);
    void setLocateMode(bool flag);
    void setTopCandidateNum(int nn);
    Eigen::Matrix4d getTransform3DoF(const int& id_pt, const int& id_theta);
public:
    void locateByFastSearch(Mat44dVec& pose_vec);
    void locateByBruteForce(Mat44dVec& pose_vec);
    CloudPtr _cloud_candi; 
    BitsetVecVec _descs_map; 
    int _n_theta = 0;
    int _n_tree = 4; 
    int _n_node = 32; 
    int _n_top = 1;

    // 1: fast search, 0: brute force
    bool _flag_locate = true; 

    Bitset _desc_src; 
    cv::Mat _mat_map; 
    cv::Mat _mat_src; 
    cv::flann::Index _tree_mat_map;

    Eigen::Matrix4d _trans_scan_to_map; 
    // int _id_candi = 0;
    // int _id_theta = 0;

    // std::vector<int> _id_pt_vec;
    // std::vector<int> _id_theta_vec;
};
#endif /*TGL_LOCALIZER_BASE_H_*/