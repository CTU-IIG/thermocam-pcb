#include "thermo_img.hpp"
#include "heat-sources.hpp"
#include "webserver.hpp"
#include "CameraCenter.h"

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

#include "config.h"

using namespace cv;

#define CAM_FPS 9 // The camera is 9Hz

/* Global switches */
bool gui_available;
bool signal_received = false;

/* Webserver globals */
Webserver *webserver = nullptr;

enum draw_mode { FULL, TEMP, NUM };
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

// Prints vector of strings at given point in a column
// Workaround for the missing support of linebreaks from OpenCV
void imgPrintStrings(Mat& img, vector<string> strings, Point2f p, Scalar color)
{
    int fontHeight = 15;
    int thickness = -1;
    int linestyle = cv::LINE_AA;

    int bl = 0;
    Size sz = ft2->getTextSize("A", fontHeight, thickness, &bl);
    for (string s : strings) {
        ft2->putText(img, s, p, fontHeight, color, thickness, linestyle, false);
        p.y += sz.height * 15 / 10;
    }
}

int getSidebarWidth(vector<POI> poi)
{
    int max = 0; // max text width
    int bl = 0; // baseline
    for (unsigned i = 0; i < poi.size(); i++) {
        string s = "00: " + poi[i].name + " 000.00 C";
        Size sz = getTextSize(s, FONT_HERSHEY_COMPLEX_SMALL, 1, 2, &bl);
        max = (sz.width > max) ? sz.width : max;
    }
    return max;
}

void drawSidebar(Mat& img, vector<POI> poi)
{
    // Draw sidebar
    int sbw = getSidebarWidth(poi);
    copyMakeBorder(img, img, 0, 0, 0, sbw, BORDER_CONSTANT, Scalar(255, 255, 255));

    // Print point names and temperatures
    vector<string> s(poi.size());
    for (unsigned i = 0; i < poi.size(); i++)
        s[i] = to_string(i) + ": " + poi[i].to_string() + "°C";
    Point2f print_coords = { (float)(img.cols - sbw + 5), 0 };
    imgPrintStrings(img, s, print_coords, Scalar(0, 0, 0));
}

// Only draw points into red channel, so original image can be retrieved
Mat drawPOI(Mat in, vector<POI> poi, draw_mode mode)
{

    Mat R;

    R = in.clone();
    if (R.channels() == 1)
        cvtColor(R, R, COLOR_GRAY2BGR);

    resize(R, R, Size(), 2, 2); // Enlarge image 2x for better looking fonts
    if (mode == NUM)
        drawSidebar(R, poi);

    for (unsigned i = 0; i < poi.size(); i++) {
        poi[i].p *= 2; // Rescale points with image
        circle(R, poi[i].p, 3, Scalar(0,255,0), -1); // Dot at POI position

        // Print point labels
        vector<string> label;
        switch (mode) {
        case FULL:
            label = { poi[i].name, poi[i].to_string(false).append("°C") };
            break;
        case TEMP:
            label = { poi[i].to_string(false).append("°C") };
            break;
        case NUM:
            label = { to_string(i) };
            break;
        }
        imgPrintStrings(R, label, poi[i].p + Point2f(3, 6), Scalar(0,0,0));
        imgPrintStrings(R, label, poi[i].p + Point2f(2, 5), Scalar(0,255,0));
    }

    return R;
}

void highlight_core(im_status &s, Mat &image){
    for(unsigned i = 0; i < s.heat_sources_border.size(); i++){
        line(image, s.heat_sources_border[i] * 2, s.heat_sources_border[(i + 1) % s.heat_sources_border.size()] * 2,
                Scalar(0,0,255));
    }
}

void updateImStatus(im_status &s, img_stream &is, const im_status &ref, bool tracking_on)
{
    s.update(is);

    if(s.poi.size() == 0) {
        s.poi = ref.poi;
        s.heat_sources_border = ref.heat_sources_border;
    }

    if (tracking_on) {
        s.updateKpDesc();
        s.updatePOICoords(ref);
    }


    for (POI& point : s.poi)
        point.temp = s.get_temperature((Point)point.p);

}

void setRefStatus(im_status &ref, img_stream &is, string poi_filename, bool tracking_on, string heat_sources_border_points)
{
    if (poi_filename.empty()) {
        ref.update(is);
    } else {
        ref.read_from_poi_json(poi_filename, heat_sources_border_points);
    }
    if (tracking_on) {
        ref.updateKpDesc();
        ref.trainMatcher(); // train once on reference image
    }
}

void onMouse(int event, int x, int y, int flags, void *param)
{
    if (event == EVENT_LBUTTONDOWN) {
        vector<POI> &poi = *((vector<POI> *)(param));
        string name = "Point " + to_string(poi.size());
        // The image is upscaled 2x when displaying POI
        // Thus we need to divide coords by 2 when getting mouse input
        poi.push_back({ name, { (float)x/2, (float)y/2 } });
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

    if (file.empty()) {
        cout << "Temperature of points of interest:";
        for (unsigned i = 0; i < poi.size(); i++)
            cout << " " << poi[i].to_string();
        cout << endl;
    } else {
        string str;
        if (access(file.c_str(), F_OK) == -1) // File does not exist
            str += csvHeaderPOI(poi);
        str += csvRowPOI(poi);
        ofstream f(file, ofstream::app);
        f << str;
        f.close();
    }
}

void showPOIImg(string path){
    im_status img;
    img.read_from_poi_json(path);
    Mat imdraw = drawPOI(img.gray, img.poi, draw_mode::NUM);
    string title = "POI from " + path;
    imshow(title,imdraw);
    waitKey(0);
    destroyAllWindows();
}

void processNextFrame(img_stream &is, const im_status &ref, im_status &curr,
                      string window_name, VideoWriter *vw,
                      string poi_csv_file, bool tracking_on)
{
    updateImStatus(curr, is, ref, tracking_on);
    printPOITemp(curr.poi, poi_csv_file);

    vector<HeatSource> hs;
    Mat laplacian, hsImg, detail, hsAvg;
    if (curr.heat_sources_border.size() > 0) {
        hs = heatSources(curr, laplacian, hsImg, detail, hsAvg, ft2);
    }

    Mat img = drawPOI(curr.gray, curr.poi, curr_draw_mode);
    highlight_core(curr, img);

    if (vw)
        vw->write(tracking_on ? img : curr.gray);

    if (webserver) {
        webserver->setImg(img);
        if (!laplacian.empty())
            webserver->setLaplacian(laplacian);
        if (!hsImg.empty())
            webserver->setHSImg(hsImg);
        if (!detail.empty())
            webserver->setDetail(detail);
        if (!hsAvg.empty()){
            webserver->setHsAvg(hsAvg);
        }
        webserver->setPOI(curr.poi);
        webserver->setHeatSources(hs);
        webserver->setCameraComponentTemps(is.getCameraComponentTemps());
        webserver->noticeClients();
    }

    if (gui_available) {
        if (!detail.empty()) {
            vector<Mat*> mats({ &detail, &laplacian, &hsImg, &hsAvg });
            int h = img.rows, w = img.cols;
            int hh = accumulate(begin(mats), end(mats), 0, [](int a, Mat *m){return max(a, m->rows + 1);});
            int ww = accumulate(begin(mats), end(mats), 0, [](int a, Mat *m){return a + m->cols + 1;});

            copyMakeBorder(img, img, 0, hh, 0, max(0, ww - w), cv::BORDER_CONSTANT,
                           Scalar(255,255,255));
            int x = 0;
            for (Mat *m : mats) {
                if (m->channels() == 1)
                    cvtColor(*m, *m, COLOR_GRAY2BGR);
                m->copyTo(img(Rect(Point(x, h + 1), m->size())));
                x += m->cols + 1;
            }
        }
        imshow(window_name, img);
    }
}

bool handle_input(bool enter_poi, im_status &ref)
{
    bool is_exit = false;

    if (gui_available) {
        char key = waitKey(1) & 0xEFFFFF;
        if (key == 8 && enter_poi && ref.poi.size() > 0) // Backspace
            ref.poi.pop_back();
        if (key == 9) // Tab
            curr_draw_mode = next(curr_draw_mode);
        if (key == 27) // Esc
            is_exit = true;
    }

    if (signal_received || (webserver && webserver->finished))
        is_exit = true;

    return is_exit;
}

void processStream(img_stream &is, im_status &ref, im_status &curr, cmd_arguments &args)
{
    int exit = 0;
    VideoWriter *vw = nullptr;
    string window_name = "Thermocam-PCB";
    chrono::time_point<chrono::system_clock> save_img_clk;

    if (gui_available)
        namedWindow(window_name, WINDOW_NORMAL);
    if (gui_available && args.enter_poi)
        setMouseCallback(window_name, onMouse, &ref.poi);
    if (!args.vid_out_path.empty()) {
        int scale = args.tracking != cmd_arguments::tracking::off ? 2 : 1;
        bool isColor = args.tracking != cmd_arguments::tracking::off;
        string cc = args.fourcc;
        vw = new VideoWriter(args.vid_out_path,
                             cv::VideoWriter::fourcc(cc[0], cc[1], cc[2], cc[3]),
                CAM_FPS, Size(ref.width*scale, ref.height*scale),
                isColor);
        if (!vw->isOpened()) {
            warnx("VideoWriter for %s not available", args.vid_out_path.c_str());
            return;
        }
    }

    bool watchdog_enabled = sd_watchdog_enabled(true, NULL) > 0;

    if (args.save_img)
        save_img_clk = chrono::system_clock::now();

    bool tracking_on = args.tracking != cmd_arguments::tracking::off;

    while (!exit) {
        if (watchdog_enabled)
            sd_notify(false, "WATCHDOG=1");

        auto begin = chrono::system_clock::now();
        processNextFrame(is, ref, curr, window_name, vw,
                         args.poi_csv_file, tracking_on);
        auto end = chrono::system_clock::now();

        if (args.tracking == cmd_arguments::tracking::once)
            tracking_on = false;

        exit = handle_input(args.enter_poi, ref);

        if (args.save_img &&
                duration_us(save_img_clk, end) > args.save_img_period * 1000000) {
            save_img_clk = chrono::system_clock::now();
            string img_path = args.save_img_dir + "/"
                              + clkDateTimeString(save_img_clk) + ".png";
            imwrite(img_path, curr.gray);
            imwrite(args.save_img_dir + "/raw_" + clkDateTimeString(save_img_clk) + ".png", curr.rawtemp);
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
    const char *font;
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
        showPOIImg(args.show_poi_path);
        exit(0);
    }

    img_stream is(args.vid_in_path, args.license_dir);
    im_status ref, curr;
    setRefStatus(ref, is, args.poi_import_path, args.tracking != cmd_arguments::tracking::off,
                 args.heat_sources_border_points);

    if (args.webserver_active)
        webserver = new Webserver();

    processStream(is, ref, curr, args);

    if (!args.poi_export_path.empty())
        curr.write_poi_json(args.poi_export_path, true);

    if (webserver)
        webserver->terminate();

    return 0;
}
