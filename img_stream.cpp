#include "img_stream.hpp"
#include <err.h>
#include <opencv2/imgproc.hpp>

using namespace cv;

img_stream::img_stream(string vid_in_path, string license_dir)
    : is_video(!vid_in_path.empty())
    , cc(init_camera_center(license_dir))
    , camera(init_camera())
    // In case of video, set the same values as those returned by our camera
    , min_rawtemp(is_video ? 7231 : findRawtempC(RECORD_MIN_C))
    , max_rawtemp(is_video ? 9799 : findRawtempC(RECORD_MAX_C))
{
    if (is_video) {
        if (access(vid_in_path.c_str(), F_OK) == -1) // File does not exist
            throw runtime_error("Video open: " + vid_in_path);
        video = new VideoCapture(vid_in_path);
    } else {
        camera->startAcquisition();
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

void img_stream::get_image(Mat_<uint16_t> &result)
{
    if (is_video) {
        Mat gray; // 8-bit gray
        *video >> gray;
        if (gray.empty()) {
            // loop to start
            video->set(cv::CAP_PROP_POS_FRAMES, 0);
            *video >> gray;
        }
        cvtColor(gray, gray, COLOR_RGB2GRAY);
        gray.convertTo(result, CV_16U,
                       (max_rawtemp - min_rawtemp)/256.0,
                       min_rawtemp);
    } else {
        if (camera->getSettings()->doNothing() < 0) {
            camera->disconnect();
            err(1,"Lost connection to camera, exiting.");
        }
        int height = camera->getSettings()->getResolutionY();
        int width = camera->getSettings()->getResolutionX();
        uint16_t *tmp = (uint16_t *)camera->retrieveBuffer();
        result.create(height, width);
        memcpy(result.data, tmp, result.total()*result.elemSize());
        camera->releaseBuffer();

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
}

double img_stream::get_temperature(uint16_t pixel_value)
{
    if (is_video)
        return RECORD_MIN_C +
                (RECORD_MAX_C - RECORD_MIN_C) *
                double(pixel_value - min_rawtemp) / (max_rawtemp - min_rawtemp);
    else
        return camera->calculateTemperatureC(pixel_value);
}

CameraCenter *img_stream::init_camera_center(string license_dir)
{
    return is_video ? nullptr : new CameraCenter(license_dir);
}

Camera *img_stream::init_camera()
{
    if (is_video)
        return nullptr;

    if (!cc || cc->getCameras().size() == 0)
        err(1,"No camera found");

    Camera *c = cc->getCameras().at(0);
    if (!c || c->connect() != 0)
        errx(1,"Error connecting camera");

    return c;
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

