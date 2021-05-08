#include "webserver.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <err.h>
#include "index.html.hpp"

void sendPOITemp(crow::response &res, std::vector<POI> poi)
{
    std::stringstream ss; 
    for (auto p : poi)
        ss << p.name << "=" << std::fixed << std::setprecision(2) << p.temp << "\n";

    res.write(ss.str());
}

void sendHeatSources(crow::response &res, std::vector<HeatSource> poi)
{
    std::stringstream ss;
    ss << "heat_sources=";
    for (auto p : poi)
        ss << p.location.x << "," << p.location.y << "," <<
              std::fixed << std::setprecision(2) << p.neg_laplacian << ";";
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

void sendPOIPosStd(crow::response &res, std::vector<POI> poi)
{
    std::stringstream ss;
    for (auto p : poi)
        ss << p.name << "=" << std::fixed << std::setprecision(4) << p.rolling_std << "\n";

    res.write(ss.str());
}

Webserver::Webserver()
    : web_thread(&Webserver::start, this)
{}

void Webserver::terminate()
{
    if (!finished)
        pthread_kill(web_thread.native_handle(), SIGINT);
    web_thread.join();
}

void Webserver::update(const cv::Mat &img,
                       const cv::Mat &detail,
                       const cv::Mat &laplacian,
                       const cv::Mat &hs_img,
                       const std::array<cv::Mat, 3> &hs_avg,
                       const std::vector<POI> &poi,
                       const std::vector<HeatSource> &hs)
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
    }
    noticeClients();
}

void Webserver::update_temps(const std::vector<std::pair<string, double> > &cct)
{
    std::lock_guard<std::mutex> lk(lock);
    this->cameraComponentTemps = cct;
}

std::string Webserver::getHeatSourcesJsonArray()
{
    stringstream ss;
    ss << "[";
    {
        ss << std::fixed << std::setprecision(3);
        std::lock_guard<std::mutex> lk(lock);
        for (auto p : heat_sources)
            ss << "[" << p.location.x << "," << p.location.y << "," << p.neg_laplacian << "],";
    }
    std::string hs = ss.str();
    hs.pop_back();
    hs += "]";

    return hs;
}

void Webserver::noticeClients() {
    std::string hs = getHeatSourcesJsonArray();
    string msg = "{\"type\":\"update\",\"heat_sources\":"s + hs + "}";

    std::lock_guard<std::mutex> _(this->usr_mtx);

    for(crow::websocket::connection* u : this->users) {
        u->send_text(msg);
    }
}

crow::response Webserver::send_jpeg(const cv::Mat &img)
{
    lock.lock();
    cv::Mat curr_img = img;
    lock.unlock();

    crow::response res;
    res.add_header("Cache-Control", "no-store");        // Images should always be fresh.
    std::vector<uchar> img_v;
    cv::imencode(".jpg", img, img_v);
    std::string img_s(img_v.begin(), img_v.end());
    res.write(img_s);
    return res;
}

void Webserver::start()
{
    crow::SimpleApp app;
    crow::mustache::set_base(".");
    app.loglevel(crow::LogLevel::Warning);

    CROW_ROUTE(app, "/")
            ([]{
        return index_html;
    });

    CROW_ROUTE(app, "/thermocam-current.jpg")
            ([this](){return send_jpeg(img);});

    CROW_ROUTE(app, "/heat_sources-current.jpg")
            ([this](){return send_jpeg(hs_img);});

    CROW_ROUTE(app, "/laplacian-current.jpg")
            ([this](){return send_jpeg(laplacian_img);});

    CROW_ROUTE(app, "/detail-current.jpg")
            ([this](){return send_jpeg(detail_img);});

    CROW_ROUTE(app, "/hs-avg<uint>.jpg")
            ([this](unsigned idx){
                if (idx >= hs_avg.size())
                    return crow::response(404);
                return send_jpeg(hs_avg[idx]);
            });

    CROW_ROUTE(app, "/temperatures.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<POI> curr_poi = this->poi;
        std::vector<std::pair<std::string,double>> curr_cct = this->cameraComponentTemps;
        this->lock.unlock();
        sendPOITemp(res, curr_poi);
        sendCameraComponentTemps(res, curr_cct);
        res.end();
    });

    CROW_ROUTE(app, "/heat-sources.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<HeatSource> curr_heat_sources = this->heat_sources;
        this->lock.unlock();
        sendHeatSources(res, curr_heat_sources);
        res.end();
    });

    CROW_ROUTE(app, "/points.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<POI> curr_poi = this->poi;
        std::vector<std::pair<std::string,double>> curr_cct = this->cameraComponentTemps;
        std::vector<HeatSource> curr_heat_sources = this->heat_sources;
        this->lock.unlock();
        sendPOITemp(res, curr_poi);
        sendCameraComponentTemps(res, curr_cct);
        sendHeatSources(res, curr_heat_sources);
        res.end();
    });

    CROW_ROUTE(app, "/position-std.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<POI> curr_poi = this->poi;
        this->lock.unlock();
        sendPOIPosStd(res, curr_poi);
        res.end();
    });


    CROW_ROUTE(app, "/ws")
            .websocket()
            .onopen([&](crow::websocket::connection& conn){
                std::cout << "New websocket connection." << std::endl;
                std::lock_guard<std::mutex> _(this->usr_mtx);
                this->users.insert(&conn);
            })
            .onclose([&](crow::websocket::connection& conn, const std::string& reason){
                std::cout << "Websocket connection closed." << std::endl;
                std::lock_guard<std::mutex> _(this->usr_mtx);
                this->users.erase(&conn);
            });

    app.port(8080)
        .multithreaded()
        .run();

    this->finished = true;
    std::cout << "Shutting down webserver thread" << std::endl;
}
