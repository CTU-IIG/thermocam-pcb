#include "thermo_img.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <err.h>

using namespace std;
using namespace cv;

string poi::to_string(bool print_name)
{
    stringstream ss;
    if (print_name)
        ss << name << "=";
    ss << fixed << setprecision(2) << temp;
    return ss.str();
}

void im_status::update(img_stream &is)
{
    if (!height || !width)
        is.get_height_width(height, width);

    is.get_image(rawtemp);

    rawtemp.convertTo(gray, CV_8U,
		      255.0 / (is.max_rawtemp - is.min_rawtemp),
		      255.0 / (1.0 - double(is.max_rawtemp) / double(is.min_rawtemp)));
}
