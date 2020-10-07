#include "webserver.hpp"
#include "crow_all.h"
#include <opencv2/highgui/highgui.hpp>

const std::string html_code =
R"(
<!DOCTYPE html>
<html>
    <head>
        <meta charset="utf-8" />
        <title>Thermocam-PCB</title>
    </head>
    <body onload="setInterval(reloadImg, 330);">
      <h2>Thermocam-PCB</h2>
      <img src="thermocam-current.jpg" id=camera />
      <script>
    var counter = 0;
    function reloadImg() {
        document.getElementById('camera').src='thermocam-current.jpg?c=' + counter++;
    }
      </script>
    </body>
</html>
)";

void sendPOITemp(crow::response &res, std::vector<poi> POI)
{
    std::stringstream ss; 
    for (auto p : POI)
        ss << "Pt." << p.name << "=" << std::fixed << std::setprecision(2) << p.temp << "\n";

    res.write(ss.str());
}

void sendHeatSources(crow::response &res, std::vector<poi> POI)
{
    std::stringstream ss;
    for (auto p : POI)
        ss << "HS." << p.p.x << "," << p.p.y << "," << p.neg_laplacian << ";";
    std::string s = ss.str();
    s.pop_back();
    s += "\n";

    res.write(s);
}

void sendCameraComponentTemps(crow::response &res, std::vector<std::pair<std::string, double>> cameraComponentTemps)
{
    std::stringstream ss;
    for (auto el : cameraComponentTemps)
        ss << el.first  << "=" << std::fixed << std::setprecision(2) << el.second << "\n";

    res.write(ss.str());
}

void sendPOIPosStd(crow::response &res, std::vector<poi> POI)
{
    std::stringstream ss;
    for (auto p : POI)
        ss << p.name << "=" << std::fixed << std::setprecision(4) << p.rolling_std << "\n";

    res.write(ss.str());
}

void sendImg(crow::response &res, cv::Mat img)
{
    std::vector<uchar> img_v;
    cv::imencode(".jpg", img, img_v);
    std::string img_s(img_v.begin(), img_v.end());
    res.write(img_s);
}

void* Webserver::start(void*)
{
    crow::SimpleApp app;
    crow::mustache::set_base(".");

    CROW_ROUTE(app, "/")
    ([]{
        return html_code;
    });

    CROW_ROUTE(app, "/thermocam-current.jpg")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        cv::Mat curr_img = this->img;
        this->lock.unlock();
        sendImg(res,curr_img);
        res.end();
    });

    CROW_ROUTE(app, "/points.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<poi> curr_POI = this->POI;
        std::vector<poi> curr_heat_sources = this->heat_sources;
        std::vector<std::pair<std::string,double>> curr_cct = this->cameraComponentTemps;
        this->lock.unlock();
        sendPOITemp(res, curr_POI);
        sendCameraComponentTemps(res, curr_cct);
        sendHeatSources(res, curr_heat_sources);
        res.end();
    });

    CROW_ROUTE(app, "/position-std.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<poi> curr_POI = this->POI;
        this->lock.unlock();
        sendPOIPosStd(res, curr_POI);
        res.end();
    });

    app.port(8080)
        .multithreaded()
        .run();

    this->finished = true;
    pthread_exit(NULL);
}
