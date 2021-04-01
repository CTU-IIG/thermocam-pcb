#ifndef THERMOCAM_PCB_HPP
#define THERMOCAM_PCB_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

// The colors 0-255 in recordings correspond to temperatures 15-120C
#define RECORD_MIN_C 15
#define RECORD_MAX_C 120

using namespace cv;

struct poi {
    std::string name;
    cv::Point2f p;
    double temp;
    double rolling_std;
    double neg_laplacian;
};

struct im_status {
    int height=0,width=0;
    uint16_t *rawtemp = nullptr;
    Mat gray;
    vector<poi> POI; // Points of interest
    vector<Point2f> heat_sources_border;
    vector<KeyPoint> kp;
    Mat desc;
};

#endif
