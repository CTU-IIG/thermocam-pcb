#ifndef WEBSERVER_HPP
#define WEBSERVER_HPP

#include <mutex>
#include <atomic>
#include "thermo_img.hpp"
#include <opencv2/core/core.hpp>
#include <thread>
#include "heat-sources.hpp"
#include <unordered_set>
#include "crow_all.h"

class Webserver
{
private:
    std::mutex lock;
    cv::Mat img;
    cv::Mat laplacian_img = {cv::Size(1, 1), CV_8U, 255};
    cv::Mat hs_img = {cv::Size(1, 1), CV_8U, 255};
    cv::Mat detail_img = {cv::Size(1, 1), CV_8U, 255};
    std::array<cv::Mat, 3> hs_avg;
    std::vector<POI> poi;
    std::vector<HeatSource> heat_sources;
    std::vector<std::pair<std::string,double>> cameraComponentTemps;
    std::unordered_set<crow::websocket::connection*> users;
    std::mutex usr_mtx;

public:
    std::atomic<bool> finished{ false };

    Webserver();
    void terminate();

    void update(const cv::Mat &img,
        const cv::Mat &detail,
        const cv::Mat &laplacian,
        const cv::Mat &hs_img,
        const std::array<cv::Mat, 3> &hs_avg,
        const std::vector<POI> &poi,
        const std::vector<HeatSource> &hs
        );

    void update_temps(const std::vector<std::pair<std::string, double>> &cct);

private:
    std::thread web_thread;

    void start();
    void noticeClients();

};

#endif
