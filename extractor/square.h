#ifndef LL_100FPS_EXTRACTOR_SQUARE_H_
#define LL_100FPS_EXTRACTOR_SQUARE_H_
#include "base/base.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
class SquareExtractor {
public:
    SquareExtractor() {}
    ~SquareExtractor() {}

    void setInputCloud(CloudPtr &cloud_in);
    void setSampleCloud(CloudPtr &cloud_in);
    void setSampleNum(const int nn);
    void setPassParas(const float min_z, const float max_z);
    void setOrigin(pcl::PointXYZ &pt);
    void setCellRows(const int nn);
    void setCellCols(const int nn);
    void setCellLength(const double length);
    
    void getSampleCloud(CloudPtr& cloud_out);
    void getDescriptorImage(cv::Mat& image_out);
    
    void segment(Bitset& bitset);
private:
    CloudPtr _cloud_src;
    CloudPtr _cloud_sam;
    pcl::PointXYZ _pt_ori;
    int _nn_sam = 8000;
    float _z_max = 2.2;
    float _z_min = -0.8;
    float _cell_len = 1.0; 
    int _rows = 40;
    int _cols = 40;
    Bitset _bitset; 
};
#endif /*LL_100FPS_EXTRACTOR_H_*/