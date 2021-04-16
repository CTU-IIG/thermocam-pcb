#ifndef IMG_STREAM_HPP
#define IMG_STREAM_HPP

#include "CameraCenter.h"
#include <opencv2/videoio.hpp>
#include <vector>
#include <utility>

// The colors 0-255 in recordings correspond to temperatures 15-120C
#define RECORD_MIN_C 15
#define RECORD_MAX_C 120

struct img_stream {
    uint16_t min_rawtemp;
    uint16_t max_rawtemp;

public:
    img_stream(std::string vid_in_path, std::string license_dir);
    ~img_stream();

    std::vector<std::pair<std::string, double>> getCameraComponentTemps();

    // TODO: Needs this to be public?
    void get_height_width(int &height, int &width);
    void get_image(cv::Mat_<uint16_t> &result);

    double get_temperature(uint16_t pixel_value);

private:
    const bool is_video;
    Camera *camera = nullptr;
    CameraCenter *cc = nullptr;
    cv::VideoCapture *video = nullptr;

    void initCamera(std::string license_dir);
    uint16_t findRawtempC(double temp);
};

#endif // IMG_STREAM_HPP
