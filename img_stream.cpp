#include "img_stream.hpp"
#include <err.h>

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
    for (raw = 0; raw < 1 << 16; raw++)
        if (camera->calculateTemperatureC(raw) >= temp)
            return raw;
}

