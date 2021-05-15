#ifndef THERMO_IMG_HPP
#define THERMO_IMG_HPP

#include <opencv2/core/mat.hpp>
#include <vector>
#include <array>
#include "img_stream.hpp"
#include <boost/accumulators/statistics/rolling_variance.hpp>
#include <opencv2/freetype.hpp>
#include <list>
#include <opencv2/imgproc.hpp>

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

enum draw_mode { FULL, TEMP, NUM };

struct thermo_img {
public:
    enum class tracking { off, copy, sync, async, finish };
    struct webimg {
        std::string name;
        std::string title;
        cv::Mat mat; // original (float) image (if any)
        cv::Mat rgb; // rgb image
        std::string html_desc;

        webimg(std::string name, std::string title, const cv::Mat &mat, std::string desc = "", enum cv::ColormapTypes cmap = cv::COLORMAP_INFERNO);
    private:
        static cv::Mat normalize(cv::Mat mat, enum cv::ColormapTypes cmap);
    };

    void update(img_stream &is);

    void draw_preview(draw_mode mode, cv::Ptr<cv::freetype::FreeType2> ft2);

    void read_from_poi_json(std::string poi_filename, std::string heat_sources_border_points = "");
    void write_poi_json(std::string path, bool verbose = false);
    void add_poi(POI &&p);
    void pop_poi();

    void trainMatcher();
    void track(const thermo_img &ref, tracking track);

    double get_temperature(uint16_t pixel);
    double get_temperature(cv::Point p);
    void updatePOICoords(const thermo_img &ref);

    const std::vector<cv::Point2f> &get_heat_sources_border() const;

    const std::vector<POI> &get_poi() const;

    cv::Mat_<uint16_t> get_rawtemp() const;
    cv::Mat get_gray() const;

    int height() const;
    int width() const;

    const std::list<webimg> &get_webimgs() const;
    const webimg *get_webimg(std::string key) const;
    const cv::Mat get_rgb(std::string key) const;

    void calcHeatSources();
    const cv::Mat get_detail() const;
    const cv::Mat get_laplacian() const;
    const cv::Mat get_hs_img() const;
    const cv::Mat get_hs_avg() const;

    const std::vector<HeatSource> &get_heat_sources() const;

    const cv::Mat &get_preview() const;

private:
    img_stream *is = nullptr;

    cv::Mat_<uint16_t> rawtemp;
    cv::Mat preview;
    cv::Mat gray;

    std::list<webimg> webimgs;
    std::vector<HeatSource> hs;

    // these values are not copied to webserver
    struct nocopy {
        nocopy() = default;
        nocopy(const nocopy &nc) {} // noop copy constructor

        std::vector<cv::KeyPoint> kp;
        cv::Mat desc;
    } nc;

    std::vector<POI> poi; // Points of interest
    std::vector<cv::Point2f> heat_sources_border;

    void updateKpDesc();
};

cv::Mat drawPOI(cv::Mat in, cv::Ptr<cv::freetype::FreeType2> ft2, std::vector<POI> poi, draw_mode mode);

#endif // THERMO_IMG_HPP
