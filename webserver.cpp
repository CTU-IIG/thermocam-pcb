#include "webserver.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <err.h>
#include <nlohmann/json.hpp>
#include "index.html.hpp"
#include "script.js.hpp"

using namespace std;
using json = nlohmann::json;

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

void Webserver::update(const thermo_img &ti)
{
    {
        std::lock_guard<std::mutex> lk(lock);
        this->ti = ti;
    }
    auto &webimgs = ti.get_webimgs();

    // webimgs are set only after the first POI tracking is calculated
    // (perhaps in background). Hence the check for size().
    if (webimgs.size() > 0 && !img_routes_initialized) {
        img_routes_initialized = true;
        for (const auto &webimg : webimgs) {
            string name = webimg.name;
            app.route_dynamic("/" + name + ".jpg")
                    ([this, name](){
                        auto img = this->ti.get_webimg(name);
                        return img ? send_img(img->rgb) : crow::response(404);
                    });
            if (webimg.mat.type() == CV_64FC1)
                app.route_dynamic("/" + name + ".tiff")
                        ([this, name](){
                            auto img = this->ti.get_webimg(name);
                            return img ? send_img(img->mat, ".tiff") : crow::response(404);
                        });
            else if (webimg.mat.type() == CV_16UC1)
                app.route_dynamic("/" + name + ".png")
                        ([this, name](){
                            auto img = this->ti.get_webimg(name);
                            return img ? send_img(img->mat, ".png") : crow::response(404);
                        });
        }
    }
    noticeClients();
}

void Webserver::update_temps(const std::vector<std::pair<string, double> > &cct)
{
    std::lock_guard<std::mutex> lk(lock);
    this->cameraComponentTemps = cct;
}

void to_json(json& j, const HeatSource& p) {
    j = json::array({p.location.x, p.location.y, int(p.neg_laplacian * 1000)/1000.0});
}

std::string Webserver::getHeatSourcesJsonArray()
{
    stringstream ss;
    ss << "[";
    {
        ss << std::fixed << std::setprecision(3);
        std::lock_guard<std::mutex> lk(lock);
        for (auto p : ti.get_heat_sources())
            ss << "[" << p.location.x << "," << p.location.y << "," << p.neg_laplacian << "],";
    }
    std::string hs = ss.str();
    hs.pop_back();
    hs += "]";

    return hs;
}

void Webserver::noticeClients() {
    json msg;
    json msg_wi = json::array();
    msg["type"] = "update";
    for (const auto& wi : ti.get_webimgs())
        msg_wi.push_back({{"name", wi.name}, {"title", wi.title}, {"desc", wi.html_desc}});
    msg["imgs"] = msg_wi;
    json msg_hs = json::array();
    for (const auto &p : ti.get_heat_sources())
        msg_hs.push_back(json(p));
    msg["heat_sources"] = msg_hs;
    
    std::lock_guard<std::mutex> _(this->usr_mtx);
    std::string msg_str = msg.dump();
    for(crow::websocket::connection* u : this->users) {
        u->send_text(msg_str);
    }
}

crow::response Webserver::send_img(const cv::Mat &img, const std::string &ext)
{
    lock.lock();
    cv::Mat curr_img = img;
    lock.unlock();

    crow::response res;
    res.add_header("Cache-Control", "no-store");        // Images should always be fresh.
    std::vector<uchar> img_v;
    cv::imencode(ext, img, img_v);
    std::string img_s(img_v.begin(), img_v.end());
    res.write(img_s);
    return res;
}

void Webserver::start()
{
    crow::mustache::set_base(".");
    app.loglevel(crow::LogLevel::Warning);

    CROW_ROUTE(app, "/")
        ([]{ return index_html; });

    CROW_ROUTE(app, "/script.js")
        ([]{
            crow::response res(script_js);
            res.set_header("Content-Type", "text/javascript");
            return res;
        });

    CROW_ROUTE(app, "/thermocam-current.jpg")
            ([this](){return send_img(ti.get_preview());});

    CROW_ROUTE(app, "/temperatures.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<POI> curr_poi = ti.get_poi();
        std::vector<std::pair<std::string,double>> curr_cct = this->cameraComponentTemps;
        this->lock.unlock();
        sendPOITemp(res, curr_poi);
        sendCameraComponentTemps(res, curr_cct);
        res.end();
    });

    CROW_ROUTE(app, "/heat-sources.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<HeatSource> curr_heat_sources = ti.get_heat_sources();
        this->lock.unlock();
        sendHeatSources(res, curr_heat_sources);
        res.end();
    });

    CROW_ROUTE(app, "/points.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<POI> curr_poi = ti.get_poi();
        std::vector<std::pair<std::string,double>> curr_cct = this->cameraComponentTemps;
        std::vector<HeatSource> curr_heat_sources = ti.get_heat_sources();
        this->lock.unlock();
        sendPOITemp(res, curr_poi);
        sendCameraComponentTemps(res, curr_cct);
        sendHeatSources(res, curr_heat_sources);
        res.end();
    });

    CROW_ROUTE(app, "/position-std.txt")
    ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        std::vector<POI> curr_poi = ti.get_poi();
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
