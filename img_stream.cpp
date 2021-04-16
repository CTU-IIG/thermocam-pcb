#include "img_stream.hpp"
#include <err.h>
#include <opencv2/imgproc.hpp>

using namespace cv;

img_stream::img_stream(string vid_in_path, string license_dir)
    : is_video(!vid_in_path.empty())
{
    if (is_video) {
        if (access(vid_in_path.c_str(), F_OK) == -1) // File does not exist
            throw runtime_error("Video open: " + vid_in_path);
        video = new VideoCapture(vid_in_path);
    } else {
        initCamera(license_dir);
        camera->startAcquisition();
        min_rawtemp = findRawtempC(RECORD_MIN_C);
        max_rawtemp = findRawtempC(RECORD_MAX_C);
    }
}

img_stream::~img_stream()
{
    camera = nullptr;
    if (is_video) {
        delete video;
    } else {
        camera->stopAcquisition();
        camera->disconnect();
        delete cc;
    }
}

std::vector<std::pair<string, double> > img_stream::getCameraComponentTemps()
{
    std::vector<std::pair<std::string, double>> v = { { "camera_shutter", 0 },
                                                      { "camera_sensor",  0 },
                                                      { "camera_housing", 0 } };
    if (!is_video) {
        v[0].second = camera->getSettings()->getShutterTemperature();
        v[1].second = camera->getSettings()->getSensorTemperature();
        v[2].second = camera->getSettings()->getHousingTemperature();
    }
    return v;
}

void img_stream::get_height_width(int &height, int &width)
{
    if (is_video) {
        height = video->get(cv::CAP_PROP_FRAME_HEIGHT);
        width = video->get(cv::CAP_PROP_FRAME_WIDTH);
    } else {
        height = camera->getSettings()->getResolutionY();
        width = camera->getSettings()->getResolutionX();
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

Mat img_stream::get_image(uint16_t *rawtemp)
{
    // TODO: Convert this to return Mat_<uint16_t> (raw temperatures)
    Mat gray; // output

    if (is_video) {
        *video >> gray;
        if (gray.empty()) {
            video->set(cv::CAP_PROP_POS_FRAMES, 0);
            *video >> gray;
        }
        cvtColor(gray, gray, COLOR_RGB2GRAY);
    } else {
        if (camera->getSettings()->doNothing() < 0) {
            camera->disconnect();
            err(1,"Lost connection to camera, exiting.");
        }
        int height, width;
        get_height_width(height, width);
        uint16_t *tmp = (uint16_t *)camera->retrieveBuffer();
        memcpy(rawtemp, tmp, height * width * sizeof(uint16_t));
        camera->releaseBuffer();
        gray = temp2gray(rawtemp, height, width, min_rawtemp, max_rawtemp);

        static uint16_t *last_buffer = NULL;
        static unsigned same_buffer_cnt = 0;
        if (tmp == last_buffer) {
            if (same_buffer_cnt++ > 10) {
                warnx("Frozen frame detected!!!!!!!!!!!!!!!!!!!!!!!!!!");
                camera->stopAcquisition();
                camera->startAcquisition();
                same_buffer_cnt = 0;
            }
        } else {
            last_buffer = tmp;
            same_buffer_cnt = 0;
        }
    }

    return gray;
}

void img_stream::initCamera(string license_dir)
{
    // Path to directory containing license file
    cc = new CameraCenter(license_dir);

    if (cc->getCameras().size() == 0)
        err(1,"No camera found");

    camera = cc->getCameras().at(0);

    if (camera->connect() != 0)
        errx(1,"Error connecting camera");
}

// Find the closest raw value corresponding to a Celsius temperature
// Linear search for simplicity
// Only use for initialization, or rewrite to log complexity!
uint16_t img_stream::findRawtempC(double temp)
{
    uint16_t raw;
    for (raw = 0; raw <= UINT16_MAX; raw++)
        if (camera->calculateTemperatureC(raw) >= temp)
            return raw;
    return UINT16_MAX;
}

