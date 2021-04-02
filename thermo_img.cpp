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

void im_status::setHeightWidth(img_stream *is)
{
    if (is->is_video) {
        height = is->video->get(cv::CAP_PROP_FRAME_HEIGHT);
        width = is->video->get(cv::CAP_PROP_FRAME_WIDTH);
    } else {
        height = is->camera->getSettings()->getResolutionY();
        width = is->camera->getSettings()->getResolutionX();
    }
}

static Mat temp2gray(uint16_t *temp, int h, int w, uint16_t min = 0, uint16_t max = 0)
{
    if (min == 0 && max == 0) {
        // Dynamic range
        max = 0;
        min = UINT16_MAX;
        for (int i = 0; i < h * w; ++i) {
            min = (min > temp[i]) ? temp[i] : min;
            max = (max < temp[i]) ? temp[i] : max;
        }
    }

    Mat G(h, w, CV_8U);
    // Write gray values
    double step = 255 / (double)(max - min); // Make pix values to 0-255
    for (int i = 0; i < h * w; ++i)
        G.data[i] = (temp[i] < min) ? 0 : (temp[i] > max) ? 255 : (temp[i] - min) * step;

    return G;
}



void im_status::update(img_stream *is)
{
    if (!height || !width)
        setHeightWidth(is);

    if (!rawtemp)
        rawtemp = new uint16_t[height * width]{ 0 };

    if (is->is_video) {
        *is->video >> gray;
        if (gray.empty()) {
            is->video->set(cv::CAP_PROP_POS_FRAMES, 0);
            *is->video >> gray;
        }
        cvtColor(gray, gray, COLOR_RGB2GRAY);
    } else {
        if (is->camera->getSettings()->doNothing() < 0) {
            is->camera->disconnect();
            err(1,"Lost connection to camera, exiting.");
        }
        uint16_t *tmp = (uint16_t *)is->camera->retrieveBuffer();
        memcpy(rawtemp, tmp, height * width * sizeof(uint16_t));
        is->camera->releaseBuffer();
        gray = temp2gray(rawtemp, height, width, is->min_rawtemp, is->max_rawtemp);

    static uint16_t *last_buffer = NULL;
    static unsigned same_buffer_cnt = 0;
    if (tmp == last_buffer) {
        if (same_buffer_cnt++ > 10) {
        warnx("Frozen frame detected!!!!!!!!!!!!!!!!!!!!!!!!!!");
        is->camera->stopAcquisition();
        is->camera->startAcquisition();
        same_buffer_cnt = 0;
        }
    } else {
        last_buffer = tmp;
        same_buffer_cnt = 0;
    }
    }
}
