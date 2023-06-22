#include "localizer/square.h"

void LocalizerSquare::loadScanDescripor(Bitset &desc_in) {
    _desc_src.reset();
    _desc_src = desc_in;
    _mat_src = cv::Mat(1, nn_cell_, CV_32FC1, 0.0);
    for (int k = 0; k < nn_cell_; ++k) {
       _mat_src.at<float>(0, k) = (float)desc_in[k];
    }
}

void LocalizerSquare::loadMapCandidatePts(CloudPtr& cloud_in) {
    _cloud_candi.reset(new PointCloud);
    *_cloud_candi = *cloud_in;
}

void LocalizerSquare::loadMapDescriptors(BitsetVecVec &descs_in) {
    _descs_map.clear();
    _descs_map = descs_in;
    
    // locate by brute force
    if (!_flag_locate) {
        return;
    }

    // locate by fast search
    cv::Mat features;
    for (int i = 0; i < _descs_map.size(); ++i) {
        for (int j = 0; j < _n_theta; ++j) {
            Bitset& desc = _descs_map[i][j];
            cv::Mat mat(1, nn_cell_, CV_32FC1, 0.0);
            for (int k = 0; k < nn_cell_; ++k) {
                mat.at<float>(0, k) = (float)desc[k];
            }
            features.push_back(mat);
        }
    }
	_mat_map = features.clone();
    _tree_mat_map.build(_mat_map, cv::flann::KDTreeIndexParams(_n_tree), cvflann::FLANN_DIST_L2); // 1
}

void LocalizerSquare::setThetaNum(int nn) {
    _n_theta = nn;
}
void LocalizerSquare::setTreeNum(int nn) {
    _n_tree = nn;
}
void LocalizerSquare::setNodeNum(int nn) {
    _n_node = nn;
}
void LocalizerSquare::setLocateMode(bool flag) {
    _flag_locate = flag;
}
void LocalizerSquare::setTopCandidateNum(int nn) {
    _n_top = nn;
}
Eigen::Matrix4d LocalizerSquare::getTransform3DoF(const int& id_pt, const int& id_theta) {
    double theta = id_theta * PI_2 / _n_theta;

    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot(0, 0) = std::cos(theta); rot(0, 1) = -std::sin(theta); 
    rot(1, 0) = std::sin(theta); rot(1, 1) =  std::cos(theta);
    Eigen::Matrix3d rot_inv = rot.inverse();

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = rot_inv;
    pose(0, 3) = _cloud_candi->points[id_pt].x;
    pose(1, 3) = _cloud_candi->points[id_pt].y;
    return pose;
}
void LocalizerSquare::startLocate(Mat44dVec& pose_vec) {
    if (_flag_locate) {
        this->locateByFastSearch(pose_vec);
    } else {
        this->locateByBruteForce(pose_vec);
    }
}

void LocalizerSquare::locateByFastSearch(Mat44dVec& pose_vec) {
    _trans_scan_to_map.setIdentity();
	std::vector<int> idx_vec(_n_top);
	std::vector<float> dist_vec(_n_top);
	cv::flann::SearchParams paras_node(_n_node);
    _tree_mat_map.knnSearch(_mat_src, idx_vec, dist_vec, _n_top, paras_node);

    // poses 3DoF 
    for (int i = 0; i < _n_top; ++i) {
        int id_pt = idx_vec[i] / _n_theta;
        int id_theta = idx_vec[i] % _n_theta;
        Eigen::Matrix4d pose = this->getTransform3DoF(id_pt, id_theta);
        pose_vec.push_back(pose);
    }

    // the best candidate
    _trans_scan_to_map = pose_vec[0];
}

void LocalizerSquare::locateByBruteForce(Mat44dVec& pose_vec) {
    std::vector<std::pair<float, std::pair<int, int>>> pairs;
    for (int i = 0; i < _cloud_candi->size(); ++i) {
        for (int j = 0; j < _n_theta; ++j) {
            float score = (_descs_map[i][j] & _desc_src).count() * 1.0 / _desc_src.count();
            pairs.push_back(std::make_pair(score, std::make_pair(i, j)));
        }
    }
    std::sort(pairs.begin(), pairs.end());
    int nn_node = pairs.size();
    for (int i = 0; i < _n_top; ++i) {
        int idx = nn_node - 1 - i;
        int id_candi = pairs[idx].second.first;
        int id_theta = pairs[idx].second.second;
        Eigen::Matrix4d pose = this->getTransform3DoF(id_candi, id_theta);
        pose_vec.push_back(pose);
    }
    // the best candidate
    _trans_scan_to_map = pose_vec[0];
}