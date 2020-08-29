#ifndef THERMOCAM_PCB_HPP
#define THERMOCAM_PCB_HPP

#include <opencv2/core/core.hpp>

struct poi {
    std::string name;
    cv::Point2f p;
    double temp;
    double rolling_std;
};

#endif
