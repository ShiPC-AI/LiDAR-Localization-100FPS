#include "base/io.h"

// binary: nn_hit_sum, nn_hit_1, nn_hit_2,... nn_hit_119, id_col(each bitset)
bool loadBitsetsColId(const std::string& filename, BitsetVec& bitset_vec,
    int nn_theta, int nn_row, int nn_col) {
    std::ifstream infile(filename, std::ios::binary);
    if (!infile.is_open()) {
        return false;
    }    
    std::vector<int> nn_hit_vec(nn_theta, 0);
    infile.read((char*)nn_hit_vec.data(), sizeof(int) * nn_theta);
    
    std::vector<std::vector<int>> idx_hit_vec(nn_theta);
    for (int i = 0; i < nn_theta; ++i) {
        const int& nn_hit = nn_hit_vec[i];
        if (nn_hit == 0) {continue;}
        idx_hit_vec[i].clear();
        idx_hit_vec[i].resize(nn_hit);
        infile.read((char*)idx_hit_vec[i].data(), sizeof(int) * nn_hit);
    }
    infile.close();
    
    for (int i = 0; i < idx_hit_vec.size(); ++i) {
        if (idx_hit_vec.empty()) {continue;}
        for (int j = 0; j < idx_hit_vec[i].size(); ++j) {
            bitset_vec[i][idx_hit_vec[i][j]] = true;
        }
    }
    return true;
}

bool saveBitsetsColId(const std::string& filename, 
    const BitsetVec& bitset_round, const int nn_theta,
    const int nn_row, const int nn_col) {
    std::ofstream outfile(filename, std::ios::binary);
    if (!outfile.is_open()) {return false;}
    
    int nn_hit = 0;
    std::vector<int> nn_hit_theta(nn_theta, 0);
    for (int i = 0; i < bitset_round.size(); ++i) {
        nn_hit_theta[i] = bitset_round[i].count();
        nn_hit += bitset_round[i].count();
    }
    outfile.write((char*)nn_hit_theta.data(), sizeof(int) * nn_theta);
    
    int id_pt = 0;
    std::vector<int> data_hit(nn_hit);
    for (int i = 0; i < bitset_round.size(); ++i) {
        for (int j = 0; j < bitset_round[i].size(); ++j) {
            if (bitset_round[i][j]) {
                data_hit[id_pt] =  j;
                ++id_pt;
            }
        }
    }
    outfile.write((char*)data_hit.data(), sizeof(int) * (int)data_hit.size());
    outfile.close();
    return true;
}

bool loadMapRings(const std::string& dir_name, BitsetVecVec& map_rings,
    const int nn_pt, const int nn_theta, const int nn_row, const int nn_col) {

    map_rings.resize(nn_pt);
    for (int i = 0; i < nn_pt; ++i) {
        // if (i % 500 == 0) {
        //     std::cout << "Load " << i << "th desc\n";
        // }
        BitsetVec bitsets(nn_theta);
        for (int j = 0; j < nn_theta; ++j) {
            bitsets[j].reset();
        }
        std::string path_bitsets = dir_name + std::to_string(i) + ".dat";
        if(!loadBitsetsColId(path_bitsets, bitsets, nn_theta, nn_row, nn_col)) {
            std::cout << "Open " << i << "th map desc failed \n"; 
            return false;
        }
        map_rings[i] = bitsets;
    }
    return true;
}