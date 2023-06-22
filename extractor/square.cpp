#include "extractor/square.h"

void SquareExtractor::setInputCloud(CloudPtr &cloud_in) {
    _cloud_src.reset(new PointCloud);
    _cloud_sam.reset(new PointCloud);
    *_cloud_src = *cloud_in;
    randomSampleCloud(_cloud_src, _cloud_sam, _nn_sam);
}

void SquareExtractor::setSampleCloud(CloudPtr &cloud_in) {
    _cloud_sam.reset(new PointCloud);
    *_cloud_sam = *cloud_in;
}

void SquareExtractor::setSampleNum(const int nn) {
    _nn_sam = nn;
}

void SquareExtractor::setPassParas(const float min_z, const float max_z) {
    _z_min = min_z;
    _z_max = max_z;
}

void SquareExtractor::setOrigin(pcl::PointXYZ &pt) {
    pcl::copyPoint(pt, _pt_ori);
}

void SquareExtractor::setCellRows(const int nn) {
    _rows = nn;
}

void SquareExtractor::setCellCols(const int nn) {
    _cols = nn;
}

void SquareExtractor::setCellLength(const double length) {
    _cell_len = length;
}

void SquareExtractor::segment(Bitset& bitset) {
    _bitset.reset();
    _z_max = _pt_ori.z + _z_max;
    _z_min = _pt_ori.z + _z_min;   
    float x_max = _pt_ori.x + _rows / 2.0 * _cell_len;
    float x_min = _pt_ori.x - _rows / 2.0 * _cell_len;
    float y_max = _pt_ori.y + _cols / 2.0 * _cell_len;
    float y_min = _pt_ori.y - _cols / 2.0 * _cell_len;

    for (int i = 0; i < _cloud_sam->size(); ++i) {
        const pcl::PointXYZ &pt = _cloud_sam->points[i];
        if (pt.z > _z_max || pt.z < _z_min) {
            continue;
        }
        if (pt.x > x_max || pt.x < x_min || pt.y > y_max || pt.y < y_min) {
            continue;
        }

        int row = (x_max - pt.x) / _cell_len;
        int col = (y_max - pt.y) / _cell_len; 
        if (row < 0 || row >= _rows || col < 0 || col >= _cols) {
            continue;
        }
       _bitset[row * _cols + col] = true;
    }
    bitset = _bitset;
}

void SquareExtractor::getSampleCloud(CloudPtr& cloud_out) {
    *cloud_out = *_cloud_sam;
}

void SquareExtractor::getDescriptorImage(cv::Mat& image_out) {
    int rows_image = _rows;
    int cols_image = _cols;
    cv::Mat image(rows_image, cols_image, CV_8UC1, cv::Scalar(0));

    for (int i = 0; i < _bitset.size(); ++i) {
        if (_bitset[i]) {
            int row = i / cols_image;
            int col = i % cols_image;
            image.at<uchar>(row, col) = 240;
        } 
    }
    image_out = image.clone();
}