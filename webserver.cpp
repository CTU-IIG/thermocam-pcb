#include "webserver.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <err.h>

const std::string html_code =
R"(
<!DOCTYPE html>
<html>
    <head>
        <meta charset="utf-8" />
        <title>Thermocam-PCB</title>
    </head>
    <body>
      <div>
        <h2>Thermocam-PCB</h2>
        <img src="thermocam-current.jpg" id=camera />
      </div>

      <div style="display: flex;">
	<div style="margin-right: 1em;">
          <h2>Detail</h2>
          <img src="detail-current.jpg" id=detail />
	</div>
	<div style="margin-right: 1em;">
          <h2>Laplacian</h2>
          <img src="laplacian-current.jpg" id=laplacian />
	</div>
	<div style="margin-right: 1em;">
          <h2>Heat-sources</h2>
          <img src="heat_sources-current.jpg" id=hs />
	</div>
	<div style="margin-right: 1em;"><h2>Trace α=0.9  </h2><img src="hs-avg0.jpg" id=avg0 /></div>
	<div style="margin-right: 1em;"><h2>Trace α=0.99 </h2><img src="hs-avg1.jpg" id=avg1 /></div>
	<div style="margin-right: 1em;"><h2>Trace α=0.999</h2><img src="hs-avg2.jpg" id=avg2 /></div>
      </div>
        <script>
            var counter = 0;
            function reloadAllImages() {
              counter++;
              document.getElementById('camera').src='thermocam-current.jpg?c=' + counter;
              document.getElementById('detail').src='detail-current.jpg?c=' + counter;
              document.getElementById('laplacian').src='laplacian-current.jpg?c=' + counter;
              document.getElementById('hs').src='heat_sources-current.jpg?c=' + counter;
              document.getElementById('avg0').src='hs-avg0.jpg?c=' + counter;
              document.getElementById('avg1').src='hs-avg1.jpg?c=' + counter;
              document.getElementById('avg2').src='hs-avg2.jpg?c=' + counter;
            };

            let server = location.host;
            var wsProtocol = 'ws://';
            if (window.location.protocol === 'https:') {
                wsProtocol = 'wss://';
            }
            let socket = new WebSocket(wsProtocol + server + "/ws");

            socket.onopen = ()=>{
                console.log('open');
            }

            socket.onclose = ()=>{
                console.log('close');
            }

            socket.onmessage = (e) => {
                console.log('e');
                reloadAllImages();
            }
        </script>
    </body>
</html>
)";

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

void sendImg(crow::response &res, cv::Mat img)
{
    res.add_header("Cache-Control", "no-store");        // Images should always be fresh.
    std::vector<uchar> img_v;
    cv::imencode(".jpg", img, img_v);
    std::string img_s(img_v.begin(), img_v.end());
    res.write(img_s);
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

void Webserver::noticeClients(){
    std::lock_guard<std::mutex> _(this->usr_mtx);
    for(crow::websocket::connection* u : this->users){
        u->send_text("Actualize!");
    }
}

void Webserver::start()
{
    crow::SimpleApp app;
    crow::mustache::set_base(".");
    app.loglevel(crow::LogLevel::Warning);

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

    CROW_ROUTE(app, "/heat_sources-current.jpg")
            ([this](const crow::request& req, crow::response& res){
        this->lock.lock();
        cv::Mat curr_img = this->hs_img;
        this->lock.unlock();
        sendImg(res,curr_img);
        res.end();
    });

    CROW_ROUTE(app, "/laplacian-current.jpg")
            ([this](const crow::request& req, crow::response& res){
                this->lock.lock();
                cv::Mat curr_img = this->laplacian_img;
                this->lock.unlock();
                sendImg(res,curr_img);
                res.end();
            });

    CROW_ROUTE(app, "/detail-current.jpg")
            ([this](const crow::request& req, crow::response& res){
                this->lock.lock();
                cv::Mat curr_img = this->detail_img;
                this->lock.unlock();
                sendImg(res,curr_img);
                res.end();
            });

    CROW_ROUTE(app, "/hs-avg<uint>.jpg")
            ([this](const crow::request& req, crow::response& res, unsigned idx){
                if (idx >= hs_avg.size()) {
                    res = crow::response(404);
                    return;
                }
                this->lock.lock();
                cv::Mat curr_img = this->hs_avg[idx];
                this->lock.unlock();
                sendImg(res,curr_img);
                res.end();
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
