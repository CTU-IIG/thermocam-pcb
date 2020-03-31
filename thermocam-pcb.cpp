#include <argp.h>
#include <err.h>
#include <unistd.h>
#include <time.h>
#include <mutex>
#include <atomic>
#include <pthread.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "Base64.h"
#include "crow_all.h"

#include "CameraCenter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>
#ifdef HAVE_LEGACY_VIDEOIO
#include <opencv2/videoio/legacy/constants_c.h>
#endif

#include "version.h"

using namespace cv;
namespace pt = boost::property_tree;

// The colors 0-255 in recordings correspond to temperatures 15-120C
#define RECORD_MIN_C 15
#define RECORD_MAX_C 120
#define CAM_FPS 9 // The camera is 9Hz

/* Command line options */
struct cmd_arguments{
    bool enter_POI = false;
    string POI_export_path;
    string POI_import_path;
    string show_POI_path;
    string license_dir;
    string vid_in_path;
    string vid_out_path;
    int display_delay_us = 0;
    string poi_csv_file;
    bool save_img = false;
    string save_img_dir;
    double save_img_period = 0;
    bool webserver_active = false;
};
cmd_arguments args;

/* Global switches */
bool gui_available;
bool sigint_received = false;;

enum draw_mode { FULL, TEMP, NUM };
constexpr draw_mode next(draw_mode m)
{
    return (draw_mode)((underlying_type<draw_mode>::type(m) + 1) % 3);
}
draw_mode curr_draw_mode = FULL;

struct poi {
    string name;
    Point2f p;
    double temp;
};

struct im_status {
    int height=0,width=0;
    uint16_t *rawtemp = nullptr;
    Mat gray;
    vector<poi> POI; // Points of interest
    vector<KeyPoint> kp;
    Mat desc;
};

struct img_stream {
    bool is_video;
    Camera *camera = nullptr;
    CameraCenter *cc = nullptr;
    VideoCapture *video = nullptr;
    uint16_t min_rawtemp;
    uint16_t max_rawtemp;
};

/* Webserver globals */
mutex web_lock;
Mat web_img;
vector<poi> web_POI;
atomic<bool> web_finished(false);
pthread_t web_thread;

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
    if (signal_num == SIGINT)
        sigint_received = true;
}

string POI2Str(poi p, bool print_name = true)
{
    stringstream ss;
    if(print_name)
        ss << p.name << "=";
    ss << fixed << setprecision(2) << p.temp;
    return ss.str();
}

void sendCurrPOITemp(crow::response& res){

    string s;
    vector<poi> POI;

    unique_lock<mutex> ulock(web_lock);
    POI = web_POI;
    ulock.unlock();

    for (auto p : POI)
        s+= POI2Str(p) + "\n";

    res.write(s);
    res.end();

}

void sendCurrImage(crow::response& res){

    vector<uchar> img_v;

    unique_lock<mutex> ulock(web_lock);
    imencode(".jpg",web_img,img_v);
    ulock.unlock();

    string img_s(img_v.begin(),img_v.end());
    res.write(img_s);
    res.end();

}

void *startWebServer(void *)
{
    crow::SimpleApp app;
    crow::mustache::set_base(".");

    CROW_ROUTE(app, "/")
    ([]{
        return crow::mustache::load("thermocam-web.html").render();
    });

    CROW_ROUTE(app, "/thermocam-current.jpg")
    ([](const crow::request& req, crow::response& res){
        sendCurrImage(res);
    });

    CROW_ROUTE(app, "/temperatures.txt")
    ([](const crow::request& req, crow::response& res){
        sendCurrPOITemp(res);
    });

    app.port(8000)
        .multithreaded()
        .run();

    web_finished = true;
    pthread_exit(NULL);
}

void initCamera(string license_dir, CameraCenter*& cc, Camera*& c)
{
    // Path to directory containing license file
    cc = new CameraCenter(license_dir);

    if (cc->getCameras().size() == 0)
        err(1,"No camera found");

    c = cc->getCameras().at(0);

    if (c->Connect() != 0)
        err(1,"Error connecting camera");
}

// Find the closest raw value corresponding to a Celsius temperature
// Linear search for simplicity
// Only use for initialization, or rewrite to log complexity!
uint16_t findRawtempC(double temp, Camera *c)
{
    uint16_t raw;
    for (raw = 0; raw < 1 << 16; raw++)
        if (c->CalculateTemperatureC(raw) >= temp)
            return raw;
}

inline double pixel2Temp(uint8_t px, double min = RECORD_MIN_C,
                         double max = RECORD_MAX_C)
{
    return ((double)px) / 255 * (max - min) + min;
}

void initImgStream(img_stream *is, string vid_in_path, string license_dir)
{
    is->is_video = !vid_in_path.empty();
    if (is->is_video) {
        is->video = new VideoCapture(vid_in_path);
    } else {
        initCamera(license_dir, is->cc, is->camera);
        is->camera->StartAcquisition();
        is->min_rawtemp = findRawtempC(RECORD_MIN_C, is->camera);
        is->max_rawtemp = findRawtempC(RECORD_MAX_C, is->camera);
    }
}

void clearImgStream(img_stream *is)
{
    if (is->is_video) {
        delete is->video;
    } else {
        is->camera->StopAcquisition();
        is->camera->Disconnect();
        delete is->cc;
    }
}

Mat temp2gray(uint16_t *temp, int h, int w, uint16_t min = 0, uint16_t max = 0)
{
    if (min == 0 && max == 0) {
        // Dynamic range
        max = 0;
        min = UINT16_MAX;
        for (int i = 0; i < h * w; ++i) {
            min = (min > temp[i]) ? temp[i] : min;
            max = (max < temp[i]) ? temp[i] : max;
        }
    }

    Mat G(h, w, CV_8U);
    // Write gray values
    double step = 255 / (double)(max - min); // Make pix values to 0-255
    for (int i = 0; i < h * w; ++i)
        G.data[i] = (temp[i] < min) ? 0 : (temp[i] > max) ? 255 : (temp[i] - min) * step;

    return G;
}

// Prints vector of strings at given point in a column
// Workaround for the missing support of linebreaks from OpenCV
void imgPrintStrings(Mat& img, vector<string> strings, Point2f p, Scalar color)
{
    // p += {10,10};
    int bl = 0;
    Size sz = getTextSize("A", FONT_HERSHEY_COMPLEX_SMALL, 1, 2, &bl);
    for (string s : strings) {
        p.y += sz.height + 5;
        putText(img, s, p, FONT_HERSHEY_COMPLEX_SMALL, 1, color, 2, CV_AA);
    }
}

int getSidebarWidth(vector<poi> POI)
{
    int max = 0; // max text width
    int bl = 0; // baseline
    for (unsigned i = 0; i < POI.size(); i++) {
        string s = "00: " + POI[i].name + " 000.00 C";
        Size sz = getTextSize(s, FONT_HERSHEY_COMPLEX_SMALL, 1, 2, &bl);
        max = (sz.width > max) ? sz.width : max;
    }
    return max;
}

void drawSidebar(Mat& img, vector<poi> POI)
{
    // Draw sidebar
    int sbw = getSidebarWidth(POI);
    copyMakeBorder(img, img, 0, 0, 0, sbw, BORDER_CONSTANT, Scalar(255, 255, 255));

    // Print point names and temperatures
    vector<string> s(POI.size());
    for (unsigned i = 0; i < POI.size(); i++)
        s[i] = to_string(i) + ": " + POI2Str(POI[i]) + " C";
    Point2f print_coords = { (float)(img.cols - sbw + 5), 0 };
    imgPrintStrings(img, s, print_coords, Scalar(0, 0, 0));
}

Mat drawPOI(Mat in, vector<poi> POI, draw_mode mode,
            Scalar color = Scalar(0, 0, 255))
{

    Mat A = in.clone();

    if (A.channels() == 1)
        cvtColor(A, A, COLOR_GRAY2RGB); // So that drawing is not grayscale

    resize(A, A, Size(), 2, 2); // Enlarge image 2x for better looking fonts
    for (unsigned i = 0; i < POI.size(); i++) {
        POI[i].p *= 2; // Rescale points with image
        circle(A, POI[i].p, 4, color, -1); // Dot at POI position

        // Print point labels
        vector<string> label;
        switch (mode) {
        case FULL:
            label = { POI[i].name, POI2Str(POI[i], false).append(" C") };
            break;
        case TEMP:
            label = { POI2Str(POI[i], false).append(" C") };
            break;
        case NUM:
            label = { to_string(i) };
            break;
        }
        imgPrintStrings(A, label, POI[i].p, color);
    }

    if (mode == NUM)
        drawSidebar(A, POI);

    return A;
}

void setStatusHeightWidth(im_status *s, img_stream *is)
{
    if (is->is_video) {
        s->height = is->video->get(CV_CAP_PROP_FRAME_HEIGHT);
        s->width = is->video->get(CV_CAP_PROP_FRAME_WIDTH);
    } else {
        s->height = is->camera->GetSettings()->GetResolutionY();
        s->width = is->camera->GetSettings()->GetResolutionX();
    }
}

void updatePOITemps(im_status *s, img_stream *is)
{
    for (unsigned i = 0; i < s->POI.size(); i++) {
        if (round(s->POI[i].p.y) < 0 || round(s->POI[i].p.y) > s->height ||
            round(s->POI[i].p.x) < 0 || round(s->POI[i].p.x) > s->width) {
            cerr << s->POI[i].name << " out of image!" << endl;
            s->POI[i].temp = nan("");
            continue;
        }
        int idx = s->width * round(s->POI[i].p.y) + round(s->POI[i].p.x);
        if (is->is_video)
            s->POI[i].temp = pixel2Temp(s->gray.data[idx]);
        else
            s->POI[i].temp = is->camera->CalculateTemperatureC(s->rawtemp[idx]);
    }
}

void updateStatusImgs(im_status *s, img_stream *is)
{

    if (!s->height || !s->width)
        setStatusHeightWidth(&(*s), is);

    if (!s->rawtemp)
        s->rawtemp = new uint16_t[s->height * s->width]{ 0 };

    if (is->is_video) {
        *is->video >> s->gray;
        if (s->gray.empty()) {
            is->video->set(CV_CAP_PROP_POS_AVI_RATIO, 0);
            *is->video >> s->gray;
        }
        cvtColor(s->gray, s->gray, COLOR_RGB2GRAY);
    } else {
        uint16_t *tmp = (uint16_t *)is->camera->RetreiveBuffer();
        memcpy(s->rawtemp, tmp, s->height * s->width * sizeof(uint16_t));
        is->camera->ReleaseBuffer();
        s->gray = temp2gray(s->rawtemp, s->height, s->width, is->min_rawtemp, is->max_rawtemp);
    }
}

void clearStatusImgs(im_status *s)
{
    if (s->rawtemp) {
        delete[] s->rawtemp;
        s->rawtemp = nullptr;
    }
    s->gray.release();
}

void updateImStatus(im_status *s, img_stream *is, im_status *ref = nullptr)
{

    updateStatusImgs(s, is);

    if(ref)
        s->POI = ref->POI; // Placeholder until tracking is implemented
    updatePOITemps(s,is);

    /*
    // Calculate new coordinates of points by homography

    fillKeyDesc(curr,curr->gray, curr->mask);

    Mat H = findHomography(ref->kp, curr->kp, ref->desc, curr->desc);
    if (!H.empty()){
        // findHomography may return an empty matrix if there aren't enough points(4)
        // to estimate a homography from them.
        // In this case, the points stay the same as before.
        perspectiveTransform(ref->POI, curr->POI, H);
    }
    */
}

void writePOI(vector<poi> POI, Mat last_img, string path, bool verbose = false)
{
    pt::ptree root, POI_pt, POI_img;
    for (unsigned i = 0; i < POI.size(); i++) {
        pt::ptree elem;
        elem.put("name", POI[i].name);
        elem.put("x", POI[i].p.x);
        elem.put("y", POI[i].p.y);
        elem.put("temp", POI[i].temp);
        POI_pt.push_back(std::make_pair("", elem));
    }
    root.add_child("POI", POI_pt);

    vector<uchar> img_v;
    imencode(".jpg",last_img,img_v);
    string img_s(img_v.begin(),img_v.end());
    POI_img.put("", macaron::Base64::Encode(img_s));
    root.add_child("POI img", POI_img);

    pt::write_json(path, root);

    if (verbose)
        cout << "Points saved to " << path << endl;
}

vector<poi> readPOI(string path)
{
    vector<poi> POI;
    pt::ptree root;
    pt::read_json(path, root);
    for (pt::ptree::value_type &p : root.get_child("POI"))
        POI.push_back({ p.second.get<string>("name"),
                        { p.second.get<float>("x"), p.second.get<float>("y") },
                        p.second.get<double>("temp") });
    return POI;
}

Mat readJsonImg(string path)
{
    pt::ptree root;
    pt::read_json(path, root);
    pt::ptree img_ptree = root.get_child("POI img");
    string img_encoded =  img_ptree.get_value<string>();
    string decoded;
    if(macaron::Base64::Decode(img_encoded,decoded) != "")
        err(1,"Base64 decoding: input data size is not a multiple of 4.");
    vector<uchar> decoded_v(decoded.begin(), decoded.end());
    return imdecode(decoded_v,0);
}

void onMouse(int event, int x, int y, int flags, void *param)
{
    if (event == EVENT_LBUTTONDOWN) {
        vector<poi> &POI = *((vector<poi> *)(param));
        string name = "Point " + to_string(POI.size());
        // The image is upscaled 2x when displaying POI
        // Thus we need to divide coords by 2 when getting mouse input
        POI.push_back({ name, { (float)x/2, (float)y/2 } });
    }
}

string clkDateTimeString(chrono::time_point<chrono::system_clock> clk){
    time_t t = chrono::system_clock::to_time_t(clk);
    char buf[64];
    if (!strftime(buf, 64, "%F %T", localtime(&t)))
        err(1, "strftime");
    string s(buf);
    return s;
}

string csvHeaderPOI(vector<poi> POI){
    string s;
    s = "Time";
    for (unsigned i = 0; i < POI.size(); i++)
        s += ", " + POI[i].name + " [°C]";
    s += "\n";
    return s;
}

string csvRowPOI(vector<poi> POI){
    stringstream ss;
    ss << clkDateTimeString(chrono::system_clock::now());
    for (unsigned i = 0; i < POI.size(); i++)
        ss << ", " << fixed << setprecision(2) << POI[i].temp;
    ss << endl;
    return ss.str();
}

void printPOITemp(vector<poi> POI, string file)
{
    if (POI.size() == 0)
        return;

    if (file.empty()) {
        cout << "Temperature of points of interest:\n";
        for (unsigned i = 0; i < POI.size(); i++)
            cout << POI2Str(POI[i]) << endl;
        cout << endl;
    } else {
        string str;
        if (access(file.c_str(), F_OK) == -1) // File does not exist
            str += csvHeaderPOI(POI);
        str += csvRowPOI(POI);
        ofstream f(file, ofstream::app);
        f << str;
        f.close();
    }
}

void showPOIImg(string path){
    vector<poi> POI =  readPOI(path);
    Mat img = readJsonImg(path);
    Mat imdraw = drawPOI(img, POI, draw_mode::NUM);
    string title = "POI from " + path;
    imshow(title,imdraw);
    waitKey(0);
    destroyAllWindows();
}

int processNextFrame(img_stream *is, im_status *ref, im_status *curr,
                     string window_name, bool enter_POI, VideoWriter *vw,
                     string poi_csv_file, bool webserver_active)
{
    updateImStatus(curr, is, ref);
    printPOITemp(curr->POI, poi_csv_file);
    Mat img = drawPOI(curr->gray, curr->POI, curr_draw_mode);

    if (vw)
        vw->write(curr->gray);

    if (webserver_active) {
        unique_lock<mutex> ulock(web_lock);
        web_img = img.clone();
        web_POI = curr->POI;
        ulock.unlock();
    }

    if (gui_available) {
        imshow(window_name, img);
        char key = waitKey(1) & 0xEFFFFF;
        if (key == 8 && enter_POI && ref->POI.size() > 0) // Backspace
            ref->POI.pop_back();
        if (key == 9) // Tab
            curr_draw_mode = next(curr_draw_mode);
        if (key == 27) // Esc
            return 1;
    }

    if (sigint_received || (webserver_active && web_finished))
        return 1;

    return 0;
}

void processStream(img_stream *is, im_status *ref, im_status *curr, cmd_arguments *args)
{
    int exit = 0;
    VideoWriter *vw = nullptr;
    string window_name = "Thermocam-PCB";
    chrono::time_point<chrono::system_clock> save_img_clk;

    updateImStatus(ref,is);

    if (gui_available)
        namedWindow(window_name, WINDOW_NORMAL);
    if (gui_available && args->enter_POI)
        setMouseCallback(window_name, onMouse, &ref->POI);
    if (!args->vid_out_path.empty())
        vw = new VideoWriter(args->vid_out_path, CV_FOURCC('H', 'F', 'Y', 'U'),
                             CAM_FPS, Size(ref->width, ref->height), 0);
    if (args->save_img)
        save_img_clk = chrono::system_clock::now();

    while (!exit) {
        auto begin = chrono::system_clock::now();
        exit = processNextFrame(is, ref, curr, window_name, args->enter_POI, vw,
                                args->poi_csv_file, args->webserver_active);
        auto end = chrono::system_clock::now();

        if (args->save_img &&
            duration_us(save_img_clk, end) > args->save_img_period * 1000000) {
            save_img_clk = chrono::system_clock::now();
            string img_path = args->save_img_dir + "/"
                              + clkDateTimeString(save_img_clk) + ".png";
            imwrite(img_path, drawPOI(curr->gray, curr->POI, draw_mode::NUM));
        }
        double process_time_us = duration_us(begin, end);
        if (args->display_delay_us > process_time_us)
            usleep(args->display_delay_us - process_time_us);
    }

    if (gui_available)
        destroyAllWindows();
    if (vw)
        delete vw; // Destructor calls VideoWriter.release to close stream
}

static error_t parse_opt(int key, char *arg, struct argp_state *argp_state)
{
    switch (key) {
    case 'e':
        args.enter_POI = true;
        if(arg != NULL)
            args.POI_export_path = arg;
        break;
    case 'p':
        args.POI_import_path = arg;
        break;
    case 's':
        args.show_POI_path = arg;
        break;
    case 'l':
        args.license_dir = arg;
        break;
    case 'r':
        args.vid_out_path = arg;
        break;
    case 'v':
        args.vid_in_path = arg;
        break;
    case 'c':
        args.poi_csv_file = arg;
        break;
    case 'd':
        args.display_delay_us = atof(arg) * 1000000;
        break;
    case -1:
        args.save_img_dir = arg;
        args.save_img = true;
        break;
    case -2:
        args.save_img_period = atof(arg);
        args.save_img = true;
        break;
    case 'w':
        args.webserver_active = true;
        break;
    case ARGP_KEY_END:
        if (args.license_dir.empty())
            args.license_dir = ".";
        if (args.save_img && args.save_img_dir.empty())
            args.save_img_dir = ".";
        if (args.save_img &&args.save_img_period == 0)
            args.save_img_period = 1;
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

/* The options we understand. */
static struct argp_option options[] = {
    { "enter-poi",       'e', "FILE",        OPTION_ARG_OPTIONAL, "Enter Points of interest by hand, optionally save them to json file at supplied path." },
    { "poi-path",        'p', "FILE",        0, "Path to config file containing saved POIs." },
    { "show-poi",        's', "FILE",        0, "Show camera image taken at saving POIs." },
    { "license-dir",     'l', "FILE",        0, "Path to directory containing WIC license file.\n\".\" by default." },
    { "record-video",    'r', "FILE",        0, "Record video and store it with entered filename"},
    { "load-video",      'v', "FILE",        0, "Load and process video instead of camera feed"},
    { "csv-log",         'c', "FILE",        0, "Log temperature of POIs to a csv file instead of printing them to stdout."},
    { "save-img-dir",    -1,  "FILE",        0, "Target directory for saving an image with POIs every \"save-img-period\" seconds.\n\".\" by default."},
    { "save-img-period", -2,  "NUM",         0, "Period for saving an image with POIs to \"save-img-dir\".\n1s by default."},
    { "delay",           'd', "NUM",         0, "Set delay between each measurement/display in seconds."},
    { "webserver",       'w', 0,             0, "Start webserver to display image and temperatures."},
    { 0 } 
};

const char * argp_program_bug_address = "https://github.com/CTU-IIG/thermocam-pcb/issues";
const char * argp_program_version = "thermocam-pcb " GIT_VERSION;

/* Our argp parser. */
static struct argp argp = {
    options, parse_opt, "[--] COMMAND...",

    "Displays thermocamera image and entered points of interest and their "
    "temperature. Writes the temperatures of entered POIs to stdout." 

    "\v"

    "Requires path to directory containing WIC license file to run with camera.\n\n"

    "Controls:\n"
    "Tab                - Change view  (Full | Temperature only | Legend)\n"
    "Mouse click (left) - Enter point  (only with --enter-poi)\n"
    "Backspace          - Remove point (only with --enter-poi)\n"
    "Esc                - Exit program\n"
};

int main(int argc, char **argv)
{

    argp_parse(&argp, argc, argv, 0, 0, NULL);

    signal(SIGINT, signalHandler);
    detectDisplay();

    if (args.webserver_active) {
        int ret = pthread_create(&web_thread, nullptr, startWebServer, nullptr);
        if (ret)
            err(1, "pthread_create");
    }

    if (!args.show_POI_path.empty() && gui_available) {
        showPOIImg(args.show_POI_path);
        exit(0);
    }

    img_stream is;
    im_status ref, curr;
    initImgStream(&is, args.vid_in_path, args.license_dir);
    if (!args.POI_import_path.empty())
        ref.POI = readPOI(args.POI_import_path);

    processStream(&is, &ref, &curr, &args);

    if (!args.POI_export_path.empty())
        writePOI(curr.POI, curr.gray, args.POI_export_path, true);

    if (args.webserver_active && !web_finished)
        if (pthread_kill(web_thread, SIGINT) || pthread_join(web_thread, NULL))
            err(1, "webserver kill, join");

    clearStatusImgs(&ref);
    clearStatusImgs(&curr);
    clearImgStream(&is);

    return 0;
}
