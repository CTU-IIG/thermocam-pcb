#include "thermocam-pcb.hpp"
#include "heat-sources.hpp"
#include "webserver.hpp"
#include "CameraCenter.h"

#include <argp.h>
#include <err.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/accumulators/statistics/rolling_variance.hpp>
#include "Base64.h"
#include <systemd/sd-daemon.h>

#include "point-tracking.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include "version.h"

using namespace cv;
namespace pt = boost::property_tree;
namespace acc = boost::accumulators;

#define CAM_FPS 9 // The camera is 9Hz

enum opt {
    OPT_FOURCC = 1000,
};

/* Command line options */
struct cmd_arguments{
    bool enter_POI = false;
    string POI_export_path;
    string POI_import_path;
    string show_POI_path;
    string license_dir;
    string vid_in_path;
    string vid_out_path;
    string fourcc = "HFYU";
    int display_delay_us = 0;
    string poi_csv_file;
    bool save_img = false;
    string save_img_dir;
    double save_img_period = 0;
    bool webserver_active = false;
    bool tracking_on = false;
    string heat_sources_border_file;
};
cmd_arguments args;

/* Global switches */
bool gui_available;
bool signal_received = false;

/* Webserver globals */
Webserver *webserver = nullptr;
pthread_t web_thread;
typedef void * (*THREADFUNCPTR)(void *);

/* Rolling variance of point positions for tracking */
std::vector<acc::accumulator_set<double, acc::stats<acc::tag::rolling_variance>>> r_var;

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

string POI2Str(poi p, bool print_name = true)
{
    stringstream ss;
    if(print_name)
        ss << p.name << "=";
    ss << fixed << setprecision(2) << p.temp;
    return ss.str();
}

void initCamera(string license_dir, CameraCenter*& cc, Camera*& c)
{
    // Path to directory containing license file
    cc = new CameraCenter(license_dir);

    if (cc->getCameras().size() == 0)
        err(1,"No camera found");

    c = cc->getCameras().at(0);

    if (c->Connect() != 0)
        errx(1,"Error connecting camera");
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

void initImgStream(img_stream *is, string vid_in_path, string license_dir)
{
    is->is_video = !vid_in_path.empty();
    if (is->is_video) {
        if (access(vid_in_path.c_str(), F_OK) == -1) // File does not exist
            err(1,"Video open");
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

// Only draw points into red channel, so original image can be retrieved
Mat drawPOI(Mat in, vector<poi> POI, draw_mode mode)
{

    Mat BG, R, BGR;

    R = in.clone();
    if (R.channels() == 3)
        cvtColor(R, R, COLOR_RGB2GRAY); // We only draw to red channel

    resize(R, R, Size(), 2, 2); // Enlarge image 2x for better looking fonts
    if (mode == NUM)
        drawSidebar(R, POI);
    BG = R.clone();

    for (unsigned i = 0; i < POI.size(); i++) {
        POI[i].p *= 2; // Rescale points with image
        circle(R, POI[i].p, 4, (255,255,255), -1); // Dot at POI position

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
        imgPrintStrings(R, label, POI[i].p, (255,255,255));
    }

    vector<Mat> v = {BG,BG,R};
    merge(v, BGR);

    return BGR;
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
                        p.second.get<double>("temp"), 0, 0});
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

void setStatusHeightWidth(im_status *s, img_stream *is)
{
    if (is->is_video) {
        s->height = is->video->get(cv::CAP_PROP_FRAME_HEIGHT);
        s->width = is->video->get(cv::CAP_PROP_FRAME_WIDTH);
    } else {
        s->height = is->camera->GetSettings()->GetResolutionY();
        s->width = is->camera->GetSettings()->GetResolutionX();
    }
}

// Temperature of various camera components
std::vector<std::pair<std::string,double>> getCameraComponentTemps(img_stream *is)
{
    std::vector<std::pair<std::string, double>> v = { { "camera_shutter", 0 },
                                                      { "camera_sensor",  0 },
                                                      { "camera_housing", 0 } };
    if (!is->is_video) {
        v[0].second = is->camera->GetSettings()->GetShutterTemperature();
        v[1].second = is->camera->GetSettings()->GetSensorTemperature();
        v[2].second = is->camera->GetSettings()->GetHousingTemperature();
    }
    return v;
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
            is->video->set(cv::CAP_PROP_POS_FRAMES, 0);
            *is->video >> s->gray;
        }
        cvtColor(s->gray, s->gray, COLOR_RGB2GRAY);
    } else {
        if (is->camera->GetSettings()->DoNothing() < 0) {
            is->camera->Disconnect();
            err(1,"Lost connection to camera, exiting.");
        }
        uint16_t *tmp = (uint16_t *)is->camera->RetreiveBuffer();
        memcpy(s->rawtemp, tmp, s->height * s->width * sizeof(uint16_t));
        is->camera->ReleaseBuffer();
        s->gray = temp2gray(s->rawtemp, s->height, s->width, is->min_rawtemp, is->max_rawtemp);

        static uint16_t *last_buffer = NULL;
        static unsigned same_buffer_cnt = 0;
        if (tmp == last_buffer) {
            if (same_buffer_cnt++ > 10) {
            warnx("Frozen frame detected!!!!!!!!!!!!!!!!!!!!!!!!!!");
            is->camera->StopAcquisition();
            is->camera->StartAcquisition();
            same_buffer_cnt = 0;
            }
        } else {
            last_buffer = tmp;
            same_buffer_cnt = 0;
        }
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

void updatePOICoords(im_status *s, im_status *ref)
{
    std::vector<cv::DMatch> matches = matchToReference(s->desc);
    Mat H = findH(ref->kp, s->kp, matches);

    if (H.empty()) // Couldn't find homography - points stay the same
        return;

    for (unsigned i=0; i<s->POI.size(); i++) {
        vector<Point2f> v = { ref->POI[i].p };
        perspectiveTransform(v, v, H); // only takes vector of points as input
        s->POI[i].p = v[0];

        // Variance of sum of 2 random variables the same as sum of variances
        // So we only need to track 1 variance per point
        r_var[i](s->POI[i].p.x + s->POI[i].p.y);
        s->POI[i].rolling_std = sqrt(acc::rolling_variance(r_var[i]));
    }

    if (ref->heat_sources_border.size() > 0)
        perspectiveTransform(ref->heat_sources_border, s->heat_sources_border, H);
}

void updateKpDesc(im_status *s)
{
    Mat pre = preprocess(s->gray);
    s->kp = getKeyPoints(pre);
    s->desc = getDescriptors(pre, s->kp);
}

void updateImStatus(im_status *s, img_stream *is, im_status *ref, bool tracking_on)
{
    updateStatusImgs(s, is);

    if(s->POI.size() == 0  || !tracking_on) {
        s->POI = ref->POI;
        s->heat_sources_border = ref->heat_sources_border;
    }

    if (tracking_on) {
        updateKpDesc(s);
        updatePOICoords(s, ref);
    }


    for (poi& point : s->POI)
        point.temp = getTemp((Point)point.p, is, s);

}

void setRefStatus(im_status *s, img_stream *is, string poi_filename, bool tracking_on, string heat_sources_border_file)
{
    if (poi_filename.empty()) {
        updateStatusImgs(s, is);
    } else {
        s->gray = readJsonImg(poi_filename);
        s->POI = readPOI(poi_filename);
        setStatusHeightWidth(s, is);
    }

    if (tracking_on || !heat_sources_border_file.empty()) {
        updateKpDesc(s);
        trainMatcher(s->desc); // train once on reference image
        // Calculate point position rolling variance from last 20 images
        r_var = std::vector<acc::accumulator_set<double, acc::stats<acc::tag::rolling_variance>>>(s->POI.size(),acc::accumulator_set<double, acc::stats<acc::tag::rolling_variance>>(acc::tag::rolling_window::window_size = 20));
    }

    if (!heat_sources_border_file.empty()) {
        // Find perspective transform from heat source border to reference frame
        im_status hs;
        vector<Point2f> hs_border;
        if(!set_border_by_POI_names(heat_sources_border_file, s->POI, hs_border)) {
            // Read and transform heat source border points
            vector <poi> tmp = readPOI(heat_sources_border_file);
            hs_border = vector <Point2f>(tmp.size());
            for (unsigned i = 0; i < tmp.size(); i++)
                hs_border[i] = tmp[i].p;
            hs.gray = readJsonImg(heat_sources_border_file);
        } else {
            hs.gray = s->gray;
        }

        // Find perspective transform from heat source border to reference frame
        updateKpDesc(&hs);
        std::vector<cv::DMatch> matches = matchToReference(hs.desc);
        Mat H = findH(s->kp, hs.kp, matches);
        H = H.inv();

        perspectiveTransform(hs_border, s->heat_sources_border, H);
    }

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
        cout << "Temperature of points of interest:";
        for (unsigned i = 0; i < POI.size(); i++)
	    cout << " " << POI2Str(POI[i]);
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
                     string poi_csv_file, bool tracking_on)
{
    updateImStatus(curr, is, ref, tracking_on);
    printPOITemp(curr->POI, poi_csv_file);

    vector<poi> hs;
    vector<Mat> hs_img;
    if (curr->heat_sources_border.size() > 0)
        hs = heatSources(curr, is, hs_img);

    Mat img = drawPOI(curr->gray, curr->POI, curr_draw_mode);

    if (vw)
        vw->write(tracking_on ? img : curr->gray);

    if (webserver) {
        webserver->setImg(img);
        webserver->setLaplacian(hs_img[0]);
        webserver->setHSImg(hs_img[1]);
        webserver->setPOI(curr->POI);
        webserver->setHeatSources(hs);
        webserver->setCameraComponentTemps(getCameraComponentTemps(is));
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

    if (signal_received || (webserver && webserver->finished))
        return 1;

    return 0;
}

void processStream(img_stream *is, im_status *ref, im_status *curr, cmd_arguments *args)
{
    int exit = 0;
    VideoWriter *vw = nullptr;
    string window_name = "Thermocam-PCB";
    chrono::time_point<chrono::system_clock> save_img_clk;

    if (gui_available)
        namedWindow(window_name, WINDOW_NORMAL);
    if (gui_available && args->enter_POI)
        setMouseCallback(window_name, onMouse, &ref->POI);
    if (!args->vid_out_path.empty()) {
        int scale = args->tracking_on ? 2 : 1;
        bool isColor = args->tracking_on;
        string cc = args->fourcc;
        vw = new VideoWriter(args->vid_out_path,
                             cv::VideoWriter::fourcc(cc[0], cc[1], cc[2], cc[3]),
                             CAM_FPS, Size(ref->width*scale, ref->height*scale),
                             isColor);
        if (!vw->isOpened()) {
            warnx("VideoWriter for %s not available", args->vid_out_path.c_str());
            return;
        }
    }

    bool watchdog_enabled = sd_watchdog_enabled(true, NULL) > 0;

    if (args->save_img)
        save_img_clk = chrono::system_clock::now();

    while (!exit) {
	if (watchdog_enabled)
	    sd_notify(false, "WATCHDOG=1");

        auto begin = chrono::system_clock::now();
        exit = processNextFrame(is, ref, curr, window_name, args->enter_POI, vw,
                                args->poi_csv_file, args->tracking_on);
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
    case OPT_FOURCC:
        if (strlen(arg) != 4) {
            argp_error(argp_state, "fourcc code must have 4 characters");
            return EINVAL;
        }
        args.fourcc = arg;
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
    case 't':
        args.tracking_on = true;
        break;
    case 'h':
        args.heat_sources_border_file = arg;
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
    { "fourcc",          OPT_FOURCC, "CODE", 0, "4-letter code for video coded used by -r (e.g. MJPG, h264), default: HFYU"},
    { "load-video",      'v', "FILE",        0, "Load and process video instead of camera feed"},
    { "csv-log",         'c', "FILE",        0, "Log temperature of POIs to a csv file instead of printing them to stdout."},
    { "save-img-dir",    -1,  "FILE",        0, "Target directory for saving an image with POIs every \"save-img-period\" seconds.\n\".\" by default."},
    { "save-img-period", -2,  "NUM",         0, "Period for saving an image with POIs to \"save-img-dir\".\n1s by default."},
    { "track-points",    't', 0,             0, "Turn on tracking of points."},
    { "heat-sources",    'h', "FILE",        0, "Detect sources of heat on-chip, inside border specified in json file."},
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

    if (args.enter_POI && args.tracking_on)
        err(1,"Can't enter points and have tracking enabled at the same time!");

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    detectDisplay();

    if (!args.show_POI_path.empty() && gui_available) {
        showPOIImg(args.show_POI_path);
        exit(0);
    }

    img_stream is;
    im_status ref, curr;
    initImgStream(&is, args.vid_in_path, args.license_dir);
    setRefStatus(&ref, &is, args.POI_import_path, args.tracking_on, args.heat_sources_border_file);

    if (args.webserver_active) {
        webserver = new Webserver();
        int ret = pthread_create(&web_thread, nullptr, (THREADFUNCPTR)&Webserver::start, webserver);
        if (ret)
            err(1, "pthread_create");
    }

    processStream(&is, &ref, &curr, &args);

    if (!args.POI_export_path.empty())
        writePOI(curr.POI, curr.gray, args.POI_export_path, true);

    if (args.webserver_active && !webserver->finished)
        if (pthread_kill(web_thread, SIGINT) || pthread_join(web_thread, NULL))
            err(1, "webserver kill, join");

    clearStatusImgs(&ref);
    clearStatusImgs(&curr);
    clearImgStream(&is);

    return 0;
}
