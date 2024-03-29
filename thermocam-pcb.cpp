#include "thermo_img.hpp"
#include "webserver.hpp"

#include "arg-parse.hpp"
#include <err.h>
#include <unistd.h>
#include <time.h>
#include <fstream>
#include <systemd/sd-daemon.h>

#include "point-tracking.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/freetype.hpp>

#include <numeric>
#include <array>

#include "config.h"

using namespace cv;
using namespace std;

#define CAM_FPS 9 // The camera is 9Hz

/* Global switches */
bool gui_available;
bool signal_received = false;

/* Webserver globals */
Webserver *webserver = nullptr;

constexpr draw_mode next(draw_mode m)
{
    return (draw_mode)((underlying_type<draw_mode>::type(m) + 1) % 3);
}
draw_mode curr_draw_mode = FULL;

inline double duration_us(chrono::time_point<chrono::system_clock> begin,
                          chrono::time_point<chrono::system_clock> end)
{
    return chrono::duration_cast<chrono::microseconds>(end - begin).count();
}

void detectDisplay()
{
    gui_available = getenv("DISPLAY") != NULL;
    if (!gui_available)
        cerr << "\"DISPLAY\" environment variable not set! All graphical functions(displaying image, entering points) turned off.\n";
}

void signalHandler(int signal_num)
{
    if (signal_num == SIGINT || signal_num == SIGTERM)
        signal_received = true;
}

cv::Ptr<cv::freetype::FreeType2> ft2;

void setRefStatus(thermo_img &ref, img_stream &is, string poi_filename, bool tracking_on, string heat_sources_border_points)
{
    if (poi_filename.empty()) {
        ref.update(is);
    } else {
        ref.read_from_poi_json(poi_filename, heat_sources_border_points);
    }
    if (tracking_on) {
        ref.trainMatcher(); // train once on reference image
    }
}

void onMouse(int event, int x, int y, int flags, void *param)
{
    if (event == EVENT_LBUTTONDOWN) {
        thermo_img &ref = *reinterpret_cast<thermo_img*>(param);
        string name = "Point " + to_string(ref.get_poi().size());
        // The image is upscaled 2x when displaying POI
        // Thus we need to divide coords by 2 when getting mouse input
        ref.add_poi({ name, { (float)x/2, (float)y/2 } });
    }
}

string clkDateTimeString(chrono::time_point<chrono::system_clock> clk){
    time_t t = chrono::system_clock::to_time_t(clk);
    char buf[64];
    if (!strftime(buf, 64, "%F_%T", localtime(&t)))
        err(1, "strftime");
    string s(buf);
    return s;
}

string csvHeaderPOI(vector<POI> poi){
    string s;
    s = "Time";
    for (unsigned i = 0; i < poi.size(); i++)
        s += ", " + poi[i].name + " [°C]";
    s += "\n";
    return s;
}

string csvRowPOI(vector<POI> poi){
    stringstream ss;
    ss << clkDateTimeString(chrono::system_clock::now());
    for (unsigned i = 0; i < poi.size(); i++)
        ss << ", " << fixed << setprecision(2) << poi[i].temp;
    ss << endl;
    return ss.str();
}

void printPOITemp(vector<POI> poi, string file)
{
    if (poi.size() == 0)
        return;

    if (!file.empty()) {
        string str;
        if (access(file.c_str(), F_OK) == -1) // File does not exist
            str += csvHeaderPOI(poi);
        str += csvRowPOI(poi);
        ofstream f(file, ofstream::app);
        f << str;
        f.close();
    }
}

void processNextFrame(img_stream &is, const thermo_img &ref, thermo_img &curr,
                      string window_name, VideoWriter *vw,
                      string poi_csv_file, thermo_img::tracking track)
{
    curr.update(is);

    curr.track(ref, track);

    printPOITemp(curr.get_poi(), poi_csv_file);

    if (curr.get_heat_sources_border().size() > 0) {
        curr.calcHeatSources();
    }

    curr.draw_preview(curr_draw_mode, ft2);

    if (vw)
        vw->write(track != thermo_img::tracking::off ? curr.get_preview() : curr.get_gray());

    if (webserver) {
        webserver->update(curr);
    }

    if (gui_available) {
        Mat img = curr.get_preview();
        if (!curr.get_detail().empty()) {
            vector<Mat> mats({ curr.get_detail(), curr.get_laplacian(), curr.get_hs_img(), curr.get_hs_avg() });
            int h = img.rows, w = img.cols;
            int hh = accumulate(begin(mats), end(mats), 0, [](int a, Mat &m){return max(a, m.rows + 1);});
            int ww = accumulate(begin(mats), end(mats), 0, [](int a, Mat &m){return a + m.cols + 1;});

            copyMakeBorder(img, img, 0, hh, 0, max(0, ww - w), cv::BORDER_CONSTANT,
                           Scalar(255,255,255));
            int x = 0;
            for (Mat &m : mats) {
                if (m.channels() == 1)
                    cvtColor(m, m, COLOR_GRAY2BGR);
                m.copyTo(img(Rect(Point(x, h + 1), m.size())));
                x += m.cols + 1;
            }
        }
        imshow(window_name, img);
    }
}

bool handle_input(bool enter_poi, thermo_img &ref)
{
    bool is_exit = false;

    if (gui_available) {
        char key = waitKey(1) & 0xEFFFFF;
        if (key == 8 && enter_poi && ref.get_poi().size() > 0) // Backspace
            ref.pop_poi();
        if (key == 9) // Tab
            curr_draw_mode = next(curr_draw_mode);
        if (key == 27) // Esc
            is_exit = true;
    }

    if (signal_received || (webserver && webserver->finished))
        is_exit = true;

    return is_exit;
}

void processStream(img_stream &is, thermo_img &ref, thermo_img &curr, cmd_arguments &args)
{
    int exit = 0;
    VideoWriter *vw = nullptr;
    string window_name = "Thermocam-PCB";
    chrono::time_point<chrono::system_clock> save_img_clk;
    chrono::time_point<chrono::system_clock> cam_temp_update_time;

    if (gui_available)
        namedWindow(window_name, WINDOW_NORMAL);
    if (gui_available && args.enter_poi)
        setMouseCallback(window_name, onMouse, &ref);
    if (!args.vid_out_path.empty()) {
        int scale = args.tracking != cmd_arguments::tracking::off ? 2 : 1;
        bool isColor = args.tracking != cmd_arguments::tracking::off;
        string cc = args.fourcc;
        vw = new VideoWriter(args.vid_out_path,
                             cv::VideoWriter::fourcc(cc[0], cc[1], cc[2], cc[3]),
                CAM_FPS, Size(ref.width()*scale, ref.height()*scale),
                isColor);
        if (!vw->isOpened()) {
            warnx("VideoWriter for %s not available", args.vid_out_path.c_str());
            return;
        }
    }

    bool watchdog_enabled = sd_watchdog_enabled(true, NULL) > 0;

    if (args.save_img)
        save_img_clk = chrono::system_clock::now();

    thermo_img::tracking track = thermo_img::tracking::off;

    switch (args.tracking) {
    case cmd_arguments::tracking::off:
        track = (args.enter_poi || ref.get_heat_sources_border().size() > 0)
            ? thermo_img::tracking::copy
            : thermo_img::tracking::off;
        break;
    case cmd_arguments::tracking::on:
        track = thermo_img::tracking::sync;
        break;
    case cmd_arguments::tracking::once:
        track = thermo_img::tracking::sync;
        break;
    case cmd_arguments::tracking::background:
        track = thermo_img::tracking::async;
        break;
    }

    while (!exit || track == thermo_img::tracking::finish) {
        if (watchdog_enabled)
            sd_notify(false, "WATCHDOG=1");
        auto begin = chrono::system_clock::now();

        processNextFrame(is, ref, curr, window_name, vw,
                         args.poi_csv_file, track);

        auto end = chrono::system_clock::now();

        if (args.tracking == cmd_arguments::tracking::once)
            track = thermo_img::tracking::off;

        if (track == thermo_img::tracking::finish)
            break;

        exit = handle_input(args.enter_poi, ref);

        if (exit && track == thermo_img::tracking::async)
            track = thermo_img::tracking::finish; // Wait until async computation finishes

        if (args.save_img &&
                duration_us(save_img_clk, end) > args.save_img_period * 1000000) {
            save_img_clk = chrono::system_clock::now();
            string img_path = args.save_img_dir + "/"
                              + clkDateTimeString(save_img_clk) + ".png";
            imwrite(img_path, curr.get_gray());
            imwrite(args.save_img_dir + "/raw_" + clkDateTimeString(save_img_clk) + ".png", curr.get_rawtemp());
        }

        // Update camera internal temperatures. Since it takes
        // about 120 ms, we do it only once per minute.
        if (webserver && end - cam_temp_update_time > 1min) {
            webserver->update_temps(is.getCameraComponentTemps());
            cam_temp_update_time = end;
        }

        double process_time_us = duration_us(begin, end);
        if (args.display_delay_us > process_time_us)
            usleep(args.display_delay_us - process_time_us);
    }

    if (gui_available)
        destroyAllWindows();
    if (vw)
        delete vw; // Destructor calls VideoWriter.release to close stream
}

void init_font()
{
    bool success = false;
    const char *font = nullptr;
    ft2 = cv::freetype::createFreeType2();
    for (const char* f : {
	    PREFIX "/" DATADIR "/thermocam-pcb/DejaVuSans.ttf",
	    "./DejaVuSans.ttf",
	}) {
	try {
	    ft2->loadFontData(f, 0);
	} catch (...) {
	    font = f;
	    continue;
	}
	success = true;
	break;
    }
    if (!success)
	err(1, "Font load error: %s", font);
}

void show_pois(const cmd_arguments &cargs)
{
    bool is_exit;
    thermo_img ref;
    const auto window_name = "Thermocam-PCB: Show POI";
    ref.read_from_poi_json(cargs.show_poi_path);

    namedWindow(window_name, WINDOW_NORMAL);
    if (cargs.enter_poi)
        setMouseCallback(window_name, onMouse, &ref);

    do {
        ref.draw_preview(curr_draw_mode, ft2);
        imshow(window_name, ref.get_preview());
        is_exit = handle_input(cargs.enter_poi, ref);
    } while (!is_exit);
    if (!cargs.poi_export_path.empty())
        ref.write_poi_json(cargs.poi_export_path, true);
}

int main(int argc, char **argv)
{
    cmd_arguments args;

    argp_parse(&argp, argc, argv, 0, 0, &args);

    if (args.enter_poi && args.tracking != cmd_arguments::tracking::off)
        err(1,"Can't enter points and have tracking enabled at the same time!");

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    detectDisplay();


    init_font();

    if (!args.show_poi_path.empty() && gui_available) {
        show_pois(args);
        exit(0);
    }

    Mat_<double> compenzation_img;
    if (args.compenzation_img.size() > 0) {
        compenzation_img = cv::imread(args.compenzation_img, IMREAD_UNCHANGED);
        compenzation_img -= mean(compenzation_img);
    }

    img_stream is(args.vid_in_path, args.license_file);
    thermo_img ref, curr(compenzation_img);
    setRefStatus(ref, is, args.poi_import_path, args.tracking != cmd_arguments::tracking::off,
                 args.heat_sources_border_points);

    if (args.webserver_active)
        webserver = new Webserver(args.poi_import_path);

    processStream(is, ref, curr, args);

    if (!args.poi_export_path.empty())
        ref.write_poi_json(args.poi_export_path, true);

    if (webserver)
        webserver->terminate();

    return 0;
}
