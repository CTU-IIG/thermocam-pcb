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


im_status::~im_status()
{
    if (rawtemp) {
        delete[] rawtemp;
        rawtemp = nullptr;
    }
    gray.release();
}

void im_status::update(img_stream *is)
{
    if (!height || !width)
        is->get_height_width(height, width);

    if (!rawtemp)
        rawtemp = new uint16_t[height * width]{ 0 };

    gray = is->get_image(rawtemp);
}
