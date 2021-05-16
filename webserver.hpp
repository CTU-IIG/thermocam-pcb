#ifndef WEBSERVER_HPP
#define WEBSERVER_HPP

#include <mutex>
#include <atomic>
#include "thermo_img.hpp"
#include <opencv2/core/core.hpp>
#include <thread>
#include <unordered_set>
#include <string>
#include "crow_all.h"
#include <chrono>

class Webserver
{
private:
    std::mutex lock;
    thermo_img ti;

    std::vector<std::pair<std::string,double>> cameraComponentTemps;
    std::unordered_set<crow::websocket::connection*> users;
    std::mutex usr_mtx;

public:
    std::atomic<bool> finished{ false };

    Webserver();
    void terminate();

    void update(const thermo_img &ti);

    void update_temps(const std::vector<std::pair<std::string, double>> &cct);

private:
    std::thread web_thread;
    crow::SimpleApp app;
    bool img_routes_initialized = false;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    void start();
    void noticeClients();

    crow::response send_img(const cv::Mat &img, const std::string &ext = ".jpg");

    std::string getHeatSourcesJsonArray();
};

#endif
