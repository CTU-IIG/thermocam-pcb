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
        <script>
          var counter = 0;
          function reloadImg() {
              document.getElementById('camera').src='thermocam-current.jpg?c=' + counter++;
          }
        </script>
      </div>

      <div style="display: flex;">
	<div style="margin-right: 1em;">
          <h2>Detail</h2>
          <img src="detail-current.jpg" id=detail />
          <script>
            var counter = 0;
            function reloadImgDetail() {
            document.getElementById('detail').src='detail-current.jpg?c=' + counter++;
            }
          </script>
	</div>
	<div style="margin-right: 1em;">
          <h2>Laplacian</h2>
          <img src="laplacian-current.jpg" id=laplacian />
          <script>
            var counter = 0;
            function reloadLaplacian() {
            document.getElementById('laplacian').src='laplacian-current.jpg?c=' + counter++;
            }
          </script>
	</div>
	<div style="margin-right: 1em;">
          <h2>Heat-sources</h2>
          <img src="heat_sources-current.jpg" id=hs />
          <script>
            var counter = 0;
            function reloadImgHs() {
            document.getElementById('hs').src='heat_sources-current.jpg?c=' + counter++;
            }
          </script>
	</div>
	<div>
          <h2>Average</h2>
          <img src="hs-avg.jpg" id=avg />
          <script>
            var counter = 0;
            function reloadImgHsAvg() {
            document.getElementById('avg').src='hs-avg.jpg?c=' + counter++;
            }
          </script>
	</div>
      </div>
        <script>
            function reloadAllImages() {reloadImg(); reloadImgDetail(); reloadImgHs(); reloadLaplacian(); reloadImgHsAvg();};

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

void Webserver::start()
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

    CROW_ROUTE(app, "/hs-avg.jpg")
            ([this](const crow::request& req, crow::response& res){
                this->lock.lock();
                cv::Mat curr_img = this->hs_avg;
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
