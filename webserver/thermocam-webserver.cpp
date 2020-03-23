#include "crow_all.h"

#include <string>
#include <vector>
#include <chrono>

using namespace std;

void sendImage(crow::response& res, string filename){
    try{
        ifstream is;
        ostringstream ss;
        string path = "images/" + filename;

        is.open(path, ifstream::in);
        is.exceptions(ifstream::failbit | ifstream::badbit);

        ss << is.rdbuf();
        res.write(ss.str());
    } catch (const std::exception& e) {
        cerr << e.what() << endl;
        res.code = 404;
        res.write(filename.append(" not found"));
    }
    res.end();
}

int main()
{
    crow::SimpleApp app;
    crow::mustache::set_base(".");

    CROW_ROUTE(app, "/")
    ([]{
        crow::mustache::context ctx;
        return crow::mustache::load("thermocam-web.html").render();
    });

    CROW_ROUTE(app, "/images/<string>")
    ([](const crow::request& req, crow::response& res, string filename){
        sendImage(res,filename);
    });

    app.port(8000)
        //.multithreaded()
        .run();
}
