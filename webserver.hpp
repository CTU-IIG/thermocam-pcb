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
    cv::Mat hs_avg = {cv::Size(1, 1), CV_8U, 255};
    std::vector<POI> poi;
    std::vector<HeatSource> heat_sources;
    std::vector<std::pair<std::string,double>> cameraComponentTemps;
    std::unordered_set<crow::websocket::connection*> users;
    std::mutex usr_mtx;

public:
    std::atomic<bool> finished{ false };

    Webserver();
    void terminate();

    void update(
        cv::Mat img,
        cv::Mat detail,
        cv::Mat laplacian,
        cv::Mat hs_img,
        cv::Mat hs_avg,
        std::vector<POI> poi,
        std::vector<HeatSource> hs,
        std::vector<std::pair<std::string, double>> cct
        )
    {
        {
            std::lock_guard<std::mutex> lk(lock);
            this->img = img;
            this->detail_img = detail;
            this->laplacian_img = laplacian;
            this->hs_img = hs_img;
            this->hs_avg = hs_avg;

            this->poi = poi;
            this->heat_sources = hs;
            this->cameraComponentTemps = cct;
        }
        noticeClients();
    }

private:
    void noticeClients(){
        std::lock_guard<std::mutex> _(this->usr_mtx);
        for(crow::websocket::connection* u : this->users){
            u->send_text("Actualize!");
        }
    }

    std::thread web_thread;

    void start();
};

#endif
