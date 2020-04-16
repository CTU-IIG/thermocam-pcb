#include "webserver.hpp"
#include "crow_all.h"
#include <opencv2/highgui/highgui.hpp>

void sendPOI(crow::response &res, std::vector<poi> POI)
{
    std::stringstream ss; 
    for (auto p : POI)
        ss << p.name << "=" << std::fixed << std::setprecision(2) << p.temp << "\n";

    res.write(ss.str());
    res.end();
}

void sendImg(crow::response &res, cv::Mat img)
{
    std::vector<uchar> img_v;
    cv::imencode(".jpg", img, img_v);
    std::string img_s(img_v.begin(), img_v.end());
    res.write(img_s);
    res.end();
}

void* Webserver::start(void*)
{
    crow::SimpleApp app;
    crow::mustache::set_base(".");

    CROW_ROUTE(app, "/")
    ([]{
        return crow::mustache::load("thermocam-web.html").render();
    });

    CROW_ROUTE(app, "/thermocam-current.jpg")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        cv::Mat curr_img = this->img;
        this->lock.unlock();
        sendImg(res,curr_img);
    });

    CROW_ROUTE(app, "/temperatures.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<poi> curr_POI = this->POI;
        this->lock.unlock();
        sendPOI(res, curr_POI);
    });

    app.port(8080)
        .multithreaded()
        .run();

    this->finished = true;
    pthread_exit(NULL);
}