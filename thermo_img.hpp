#ifndef THERMO_IMG_HPP
#define THERMO_IMG_HPP

#include <opencv2/core/mat.hpp>
#include <vector>
#include <array>
#include "img_stream.hpp"
#include <boost/accumulators/statistics/rolling_variance.hpp>
#include <opencv2/freetype.hpp>

struct HeatSource {
    cv::Point location;
    double temperature;
    double neg_laplacian;
};

// Point of interrest
struct POI {
    std::string name;
    cv::Point2f p;
    double temp;
    double rolling_std;

    POI() {};
    POI(std::string name, cv::Point2f p, double temp = 0.0) : name(name), p(p), temp(temp) {};

    /* Rolling variance of point positions for tracking */
    using rolling_variance_accumulator = boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_variance>>;
    rolling_variance_accumulator r_var { boost::accumulators::tag::rolling_window::window_size = 20 };

    std::string to_string(bool print_name = true);
};

struct im_status {
public:
    enum class tracking { off, sync, async, finish };

    void update(img_stream &is);
    void read_from_poi_json(std::string poi_filename, std::string heat_sources_border_points = "");
    void write_poi_json(std::string path, bool verbose = false);
    void add_poi(POI &&p);
    void pop_poi();

    void trainMatcher();
    void track(const im_status &ref, tracking track);

    double get_temperature(uint16_t pixel);
    double get_temperature(cv::Point p);
    void updatePOICoords(const im_status &ref);

    const std::vector<cv::Point2f> &get_heat_sources_border() const;

    const std::vector<POI> &get_poi() const;

    cv::Mat_<uint16_t> get_rawtemp() const;
    cv::Mat get_gray() const;

    int height() const;
    int width() const;

    std::vector<HeatSource> heatSources(cv::Ptr<cv::freetype::FreeType2> ft2);
    const cv::Mat &get_detail() const;
    const cv::Mat &get_laplacian() const;
    const cv::Mat &get_hs_img() const;
    const std::array<cv::Mat, 3> &get_hs_avg() const;

private:
    img_stream *is = nullptr;

    cv::Mat_<uint16_t> rawtemp;
    cv::Mat gray;
    cv::Mat detail_out;
    cv::Mat laplacian_out;
    cv::Mat hsImg_out;
    std::array<cv::Mat, 3> hsAvg_out;

    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;

    std::vector<POI> poi; // Points of interest
    std::vector<cv::Point2f> heat_sources_border;

    void updateKpDesc();
};

#endif // THERMO_IMG_HPP
