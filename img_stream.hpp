#ifndef IMG_STREAM_HPP
#define IMG_STREAM_HPP

#ifdef WITH_WIC_SDK
#include "CameraCenter.h"
#endif
#include <opencv2/videoio.hpp>
#include <vector>
#include <utility>
#include <string>

// The colors 0-255 in recordings correspond to temperatures 15-120C
#define RECORD_MIN_C 15
#define RECORD_MAX_C 120

struct img_stream {
public:
    img_stream(std::string vid_in_path, std::string license_dir);
    ~img_stream();

    std::vector<std::pair<std::string, double>> getCameraComponentTemps();

    void get_image(cv::Mat_<uint16_t> &result);

    double get_temperature(uint16_t pixel_value);

private:
    char tmp; // To make #ifdef in constructor simpler
#ifdef WITH_WIC_SDK
    const
#else
    static constexpr
#endif
    bool is_video = true;
    cv::VideoCapture *video = nullptr;
#ifdef WITH_WIC_SDK
    CameraCenter *cc = nullptr;
    Camera *camera = nullptr;
#endif

public:
    const uint16_t min_rawtemp; // must be initialized after camera
    const uint16_t max_rawtemp;

private:
#ifdef WITH_WIC_SDK
    CameraCenter *init_camera_center(std::string license_dir);
    Camera *init_camera();
#endif
    uint16_t findRawtempC(double temp);
};

#endif // IMG_STREAM_HPP
