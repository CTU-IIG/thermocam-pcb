#ifndef THERMO_IMG_HPP
#define THERMO_IMG_HPP

#include <opencv2/core/mat.hpp>
#include <vector>
#include "img_stream.hpp"

struct poi {
    std::string name;
    cv::Point2f p;
    double temp;
    double rolling_std;
    double neg_laplacian;

    std::string to_string(bool print_name = true);
};

struct im_status {
    int height=0,width=0;
    uint16_t *rawtemp = nullptr;
    cv::Mat gray;
    std::vector<poi> POI; // Points of interest
    std::vector<cv::Point2f> heat_sources_border;
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;

    ~im_status();

    void setHeightWidth(img_stream *is);
    void update(img_stream *is);
};

#endif // THERMO_IMG_HPP
