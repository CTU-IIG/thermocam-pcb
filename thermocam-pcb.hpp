#ifndef THERMOCAM_PCB_HPP
#define THERMOCAM_PCB_HPP

#include <opencv2/core/core.hpp>

struct poi {
    std::string name = "";
    cv::Point2f p = {0,0};
    double temp = 0;
    double rolling_std = 0;
    double neg_laplacian = 0;
};

#endif
