#include "img_stream.hpp"
#include <err.h>
#include <opencv2/imgproc.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

using namespace cv;
using namespace std;

img_stream::img_stream(string vid_in_path, string license_file)
    : tmp(0)
#ifdef WITH_WIC_SDK
    , is_video(!vid_in_path.empty())
    , license(license_file)
    , wic(init_wic())
    , grabber(init_grabber())
#endif
    // In case of video, set the same values as those returned by our camera
    , min_rawtemp(is_video ? 7231 : findRawtempC(RECORD_MIN_C))
    , max_rawtemp(is_video ? 9799 : findRawtempC(RECORD_MAX_C))
{
    if (is_video) {
        if (access(vid_in_path.c_str(), F_OK) == -1) // File does not exist
            throw runtime_error("Video open: " + vid_in_path);
        video = new VideoCapture(vid_in_path);
    }
}

img_stream::~img_stream()
{
    if (is_video) {
        delete video;
    } else {
#ifdef WITH_WIC_SDK
        grabber = nullptr;
        delete wic;
#endif
    }
}

std::vector<std::pair<string, double> > img_stream::getCameraComponentTemps()
{
    std::vector<std::pair<std::string, double>> v = { { "camera_shutter", 0 },
                                                      { "camera_sensor",  0 },
                                                      { "camera_housing", 0 } };
    if (!is_video) {
#ifdef WITH_WIC_SDK
        v[0].second = wic->getCameraTemperature(wic::CameraTemperature::ShutterTemp).second.value_or(0.0);
        v[1].second = wic->getCameraTemperature(wic::CameraTemperature::SensorTemp).second.value_or(0.0);
        v[2].second = wic->getCameraTemperature(wic::CameraTemperature::HousingTemp).second.value_or(0.0);
#endif
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
#ifdef WITH_WIC_SDK
        if (!grabber->isConnected()) {
            err(1,"Lost connection to camera, exiting.");
        }

        // sensor temp should be polled often
	auto coreTemp = wic->getCameraTemperature(wic::CameraTemperature::SensorTemp);
        vector<uint8_t> buffer = grabber->getBuffer(1000);
        auto buffer16 = reinterpret_cast<uint16_t*>(buffer.data());
        wic->calibrateRawInplace(buffer16, buffer.size() / 2, coreTemp.second.value_or(0));

        static unsigned empty_buffer_cnt = 0;
        if (buffer.size() == 0) {
            if (empty_buffer_cnt++ > 10) {
                // Exit and let systemd restart us
                errx(1, "Empty buffer detected!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
        } else {
            empty_buffer_cnt = 0;
        }

        auto [width, height] = wic->getResolution();
        result.create(height, width);
        memcpy(result.data, buffer.data(),
               std::min(result.total()*result.elemSize(), buffer.size()));
#endif
    }
}

double img_stream::get_temperature(uint16_t pixel_value)
{
    if (is_video)
        return RECORD_MIN_C +
                (RECORD_MAX_C - RECORD_MIN_C) *
                double(pixel_value - min_rawtemp) / (max_rawtemp - min_rawtemp);
#ifdef WITH_WIC_SDK
    else {
        // TODO: Do we need to call getCurrentTemperatureResolution
        // for every pixel? Currently, it doesn't seem to be
        // performance critical, but still...
        auto tempRes = wic->getCurrentTemperatureResolution();
        return wic::rawToCelsius(pixel_value, tempRes);
    }
#endif
}

#ifdef WITH_WIC_SDK
wic::WIC *img_stream::init_wic()
{
    if (!license.isOk())
        errx(1, "wic license invalid");

    auto wic = wic::findAndConnect(license);

    auto defaultRes = wic->doDefaultWICSettings();
    if (defaultRes.first != wic::ResponseStatus::Ok) {
        std::cerr << "DoDefaultWICSettings error: "
                  << wic::responseStatusToStr(defaultRes.first) << std::endl;
        exit(1);
    }

    auto resolution = wic->getResolution();
    if (resolution.first == 0 || resolution.second == 0) {
        std::cerr << "Invalid resolution, core detection error." << std::endl;
        exit(1);
    }

    auto rangeAndLens = wic->getRangeAndLens();

    // if no lens were detected, set desired lens from license file
    // this may take a few minutes, especially with validateMemory flag on
    if (rangeAndLens.second.empty() &&
        !license.calibrationData().lensCalibrationData().empty()) {
        auto lens =
            license.calibrationData().lensCalibrationData().begin()->lens();
        cerr << "wic::setRangeAndLensRes..." << endl;
        auto setRangeAndLensRes =
            wic->setRangeAndLens(wic::Range::Low, lens, false, false);

        if (setRangeAndLensRes.second) {
            std::cerr << "Errors occurred while uploading calibration. " << wic::responseStatusToStr(setRangeAndLensRes.first)
                      << std::endl
                      << "lens: " << lens << "; error count: "
                      << setRangeAndLensRes.second.value_or(0) << " out of "
                      << 2 * resolution.first * resolution.second / 128 << endl;
            exit(1);
        }
        cerr << "done" << endl;
    }
    return is_video ? nullptr : wic;
}

wic::FrameGrabber *img_stream::init_grabber()
{
    if (is_video)
        return nullptr;

    if (!wic)
        err(1,"No camera found");

    wic::FrameGrabber *grabber = wic->frameGrabber();
    if (!grabber)
        errx(1,"Error connecting camera");

    grabber->setup();

    return grabber;
}

uint16_t img_stream::findRawtempC(double celsius)
{
    auto tempRes = wic->getCurrentTemperatureResolution();
    return wic::celsiusToRaw(celsius, tempRes);
}
#else
uint16_t img_stream::findRawtempC(double temp) { return 0; }
#endif
