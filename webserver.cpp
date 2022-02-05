#define CROW_MAIN
#include "webserver.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <err.h>
#include <nlohmann/json.hpp>
#include "index.html.hpp"
#include "script.js.hpp"
#include <filesystem>


using namespace std;
namespace fs = std::filesystem;
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

std::string get_poi_name(const std::string &poi_path)
{
    fs::path p(poi_path);
    if (fs::is_symlink(p))
        p = fs::read_symlink(p);
    return p.stem();
}

Webserver::Webserver(const std::string &poi_path)
    : poi_name(get_poi_name(poi_path))
    , web_thread(&Webserver::start, this)
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
        frame_cnt++;
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

// Called from update() - no need to lock this->lock
void Webserver::noticeClients() {
    json msg;
    json msg_lwi = json::array();
    msg["type"] = "update";
    for (const auto &lwi : ti.get_webimgs()) {
        json msg_wi = json::array();
        for (const auto& wi : lwi)
            msg_wi.push_back({{"name", wi.name}, {"title", wi.title}, {"desc", wi.html_desc}});
        msg_lwi.push_back(msg_wi);
    }
    msg["imgs"] = msg_lwi;
    json msg_hs = json::array();
    for (const auto &p : ti.get_heat_sources())
        msg_hs.push_back(json(p));
    msg["heat_sources"] = msg_hs;

    json msg_pt = json::object();
    for (const POI &poi : ti.get_poi()) {
        msg_pt[poi.name] = int(poi.temp*100)/100.0;
    }
    msg["poi_temp"] = msg_pt;

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

std::string Webserver::prometheus_metics()
{
    this->lock.lock();
    std::vector<POI> curr_poi = ti.get_poi();
    std::vector<std::pair<std::string,double>> curr_cct = this->cameraComponentTemps;
    this->lock.unlock();

    std::stringstream ss;
    ss << "# TYPE thermocam_point_temp gauge\n";
    for (auto p : curr_poi)
        ss << "thermocam_point_temp{name=\""<< poi_name <<"\", point=\""<< p.name <<"\"} " << std::fixed << std::setprecision(2) << p.temp << "\n";

    ss << "# TYPE thermocam_temp gauge\n";
    for (auto el : cameraComponentTemps)
        ss << "thermocam_temp{component=\""<< el.first <<"\"} " << std::fixed << std::setprecision(2) << el.second << "\n";

    return ss.str();
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
            .onaccept([&](const crow::request &req) {
                std::cout << "New websocket connection from " << req.remoteIpAddress << std::endl;
                return true;
            })
            .onopen([&](crow::websocket::connection& conn){
                std::lock_guard<std::mutex> _(this->usr_mtx);
                this->users.insert(&conn);
            })
            .onclose([&](crow::websocket::connection& conn, const std::string& reason){
                std::cout << "Websocket connection closed." << std::endl;
                std::lock_guard<std::mutex> _(this->usr_mtx);
                this->users.erase(&conn);
            });

    CROW_ROUTE(app, "/uptime.txt")
        ([this]() {
            using namespace std::chrono;
            steady_clock::time_point now = std::chrono::steady_clock::now();
            return to_string(duration_cast<seconds>(now - start_time).count());
        });

    CROW_ROUTE(app, "/frame.txt")
        ([this]() { return to_string(frame_cnt); });

    CROW_ROUTE(app, "/users.txt")
        ([this]() { return to_string(users.size()); });

    CROW_ROUTE(app, "/metrics")
        ([this]() { return prometheus_metics(); });

    CROW_ROUTE(app, "/<path>")
            ([this](const string &path) {
                for (const auto &webimg_list : ti.get_webimgs()) {
                    for (const auto &webimg : webimg_list) {
                        if (path == webimg.name + ".jpg") {
                            return send_img(webimg.rgb);
                        } else if (path == webimg.name + ".tiff") {
                            return send_img(webimg.mat, ".tiff");
                        };
                    }
                }
                return crow::response(404);
            });

    app.port(8080)
        .multithreaded()
        .run();

    this->finished = true;
    std::cout << "Shutting down webserver thread" << std::endl;
}
