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
}

void im_status::updateKpDesc()
{
    Mat pre = preprocess(gray);
    kp = getKeyPoints(pre);
    desc = getDescriptors(pre, kp);
}

