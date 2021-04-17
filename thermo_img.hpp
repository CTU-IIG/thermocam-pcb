#ifndef THERMO_IMG_HPP
#define THERMO_IMG_HPP

#include <opencv2/core/mat.hpp>
#include <vector>
#include "img_stream.hpp"
#include <boost/accumulators/statistics/rolling_variance.hpp>

// Point of interrest
struct POI {
    std::string name;
    cv::Point2f p;
    double temp;
    double rolling_std;
    double neg_laplacian;

    POI() {};
    POI(std::string name, cv::Point2f p, double temp = 0.0) : name(name), p(p), temp(temp) {};

    /* Rolling variance of point positions for tracking */
    using rolling_variance_accumulator = boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_variance>>;
    rolling_variance_accumulator r_var { boost::accumulators::tag::rolling_window::window_size = 20 };

    std::string to_string(bool print_name = true);
};

struct im_status {
    int height=0,width=0;
    cv::Mat_<uint16_t> rawtemp;
    cv::Mat gray;
    std::vector<POI> poi; // Points of interest
    std::vector<cv::Point2f> heat_sources_border;
    std::vector<cv::Point2f> border_frame;
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;

    void update(img_stream &is);
    void updateKpDesc();

    double get_temperature(cv::Point p);
    void updatePOICoords(const im_status &ref);
private:
    img_stream *is = nullptr;
};

#endif // THERMO_IMG_HPP
