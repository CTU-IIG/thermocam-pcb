#include "thermo_img.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <err.h>
#include "point-tracking.hpp"

using namespace std;
using namespace cv;

string POI::to_string(bool print_name)
{
    stringstream ss;
    if (print_name)
        ss << name << "=";
    ss << fixed << setprecision(2) << temp;
    return ss.str();
}

void im_status::update(img_stream &is)
{
    is.get_image(rawtemp);
    width = rawtemp.cols;
    height = rawtemp.rows;

    rawtemp.convertTo(gray, CV_8U,
                      255.0 / (is.max_rawtemp - is.min_rawtemp),
                      255.0 / (1.0 - double(is.max_rawtemp) / double(is.min_rawtemp)));

    this->is = &is;
}

void im_status::updateKpDesc()
{
    Mat pre = preprocess(gray);
    kp = getKeyPoints(pre);
    desc = getDescriptors(pre, kp);
}

double im_status::get_temperature(Point p)
{
    if (p.y < 0 || p.y > height || p.x < 0 || p.x > width) {
        cerr << "Point at (" << p.x << "," << p.y << ") out of image!" << endl;
        return nan("");
    }
    if (!is)
        return nan("");

    uint16_t pixel = rawtemp(p);
    return is->get_temperature(pixel);
}

void im_status::updatePOICoords(const im_status &ref)
{
    std::vector<cv::DMatch> matches = matchToReference(desc);
    Mat H = findH(ref.kp, kp, matches);

    if (H.empty()) // Couldn't find homography - points stay the same
        return; // FIXME: Let the caller (or at least user) know that this happened

    for (unsigned i=0; i < poi.size(); i++) {
        vector<Point2f> v = { ref.poi[i].p };
        perspectiveTransform(v, v, H); // only takes vector of points as input
        poi[i].p = v[0];

        // Variance of sum of 2 random variables the same as sum of variances
        // So we only need to track 1 variance per point
        poi[i].r_var(poi[i].p.x + poi[i].p.y);
	namespace acc = boost::accumulators;
        poi[i].rolling_std = sqrt(acc::rolling_variance(poi[i].r_var));
    }

    if (ref.heat_sources_border.size() > 0)
        perspectiveTransform(ref.heat_sources_border, heat_sources_border, H);
}

void im_status::setFixedFrame(){
    int x_max = INT_MIN, y_max = INT_MIN, x_min = INT_MAX, y_min = INT_MAX;
    for (auto el : heat_sources_border) {
        x_max = (x_max < el.x) ? el.x : x_max;
        y_max = (y_max < el.y) ? el.y : y_max;
        x_min = (x_min > el.x) ? el.x : x_min;
        y_min = (y_min > el.y) ? el.y : y_min;
    }

    border_frame = {cv::Point2f(0, 0),
                    cv::Point2f(x_max - x_min, 0),
                    cv::Point2f(x_max - x_min, y_max - y_min),
                    cv::Point2f(0, y_max - y_min)};
}
