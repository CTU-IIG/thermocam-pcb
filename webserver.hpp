#ifndef WEBSERVER_HPP
#define WEBSERVER_HPP

#include <mutex>
#include <atomic>
#include "thermo_img.hpp"
#include <opencv2/core/core.hpp>
#include <thread>
#include <unordered_set>
#include "crow_all.h"

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

    void start();
    void noticeClients();

    crow::response send_jpeg(const cv::Mat &img);

    std::string getHeatSourcesJsonArray();
};

#endif
