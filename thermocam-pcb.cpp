#include <argp.h>
#include <err.h>
#include <unistd.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "Base64.h"

#include "CameraCenter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>

#define duration_us(a) std::chrono::duration_cast<std::chrono::microseconds>(a).count()

using namespace cv;
namespace pt = boost::property_tree;

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
};

struct im_status {
    int height=0,width=0;
    uint16_t *rawtemp = NULL;
    uint16_t min_rawtemp = UINT16_MAX, max_rawtemp = 0;
    Mat gray;
    vector<poi> POI; // Points of interest
    vector<KeyPoint> kp;
    Mat desc;
};

struct img_stream {
    bool is_video;
    Camera *camera = NULL;
    CameraCenter *cc = NULL;
    VideoCapture *video = NULL;
};

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

void initImgStream(img_stream *is, string vid_in_path, string license_dir)
{
    is->is_video = !vid_in_path.empty();
    if (is->is_video) {
        is->video = new VideoCapture(vid_in_path);
    } else {
        initCamera(license_dir, is->cc, is->camera);
        is->camera->StartAcquisition();
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

vector<double> getPOITemp(vector<poi> POI, uint16_t *rawtemp, Camera *c, int max_x, int max_y)
{
    vector<double> temp(POI.size());
    for (unsigned i = 0; i < POI.size(); i++) {
        if (round(POI[i].p.y) < 0 || round(POI[i].p.y) > max_y ||
            round(POI[i].p.x) < 0 || round(POI[i].p.x) > max_x) {
            cerr << POI[i].name << " out of image!" << endl;
            temp[i] = nan("");
            continue;
        }
        int idx = max_x * round(POI[i].p.y) + round(POI[i].p.x);
        temp[i] = (c) ? c->CalculateTemperatureC(rawtemp[idx]) : nan("");
    }
    return temp;
}

string temp2str(double temp)
{
    stringstream ss;
    ss << fixed << setprecision(2) << temp << " C";
    return ss.str();
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

void drawSidebar(Mat& img, vector<poi> POI, vector<double> temp)
{
    // Draw sidebar
    int sbw = getSidebarWidth(POI);
    copyMakeBorder(img, img, 0, 0, 0, sbw, BORDER_CONSTANT, Scalar(255, 255, 255));

    // Print point names and temperatures
    vector<string> s(POI.size());
    for (unsigned i = 0; i < POI.size(); i++)
        s[i] = to_string(i) + ": " + POI[i].name + ": " + temp2str(temp[i]);
    Point2f print_coords = { (float)(img.cols - sbw + 5), 0 };
    imgPrintStrings(img, s, print_coords, Scalar(0, 0, 0));
}

Mat drawPOI(Mat &img_in, vector<poi> POI, uint16_t *curr_rawtemp, Camera *c, draw_mode mode,
             Scalar color = Scalar(0, 0, 255))
{

    Mat A = img_in.clone();

    if (A.channels() == 1)
        cvtColor(A, A, COLOR_GRAY2RGB); // So that drawing is not grayscale

    // Temperatures at POI positions from rawtemp
    vector<double> temp = getPOITemp(POI, curr_rawtemp, c, A.cols, A.rows);

    resize(A, A, Size(), 2, 2); // Enlarge image 2x for better looking fonts
    for (unsigned i = 0; i < POI.size(); i++) {
        POI[i].p *= 2; // Rescale points with image
        circle(A, POI[i].p, 4, color, -1); // Dot at POI position

        // Print point labels
        vector<string> label;
        switch (mode) {
        case FULL:
            label = { POI[i].name, temp2str(temp[i]) };
            break;
        case TEMP:
            label = { temp2str(temp[i]) };
            break;
        case NUM:
            label = { to_string(i) };
            break;
        }
        imgPrintStrings(A, label, POI[i].p, color);
    }

    if (mode == NUM)
        drawSidebar(A, POI, temp);

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

void setStatusImgs(im_status *s, img_stream *is, im_status *ref = NULL)
{

    if (!s->height || !s->width)
        setStatusHeightWidth(&(*s), is);

    if (s->rawtemp == NULL)
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
        if (ref == NULL)
            s->gray = temp2gray(s->rawtemp, s->height, s->width);
        else
            s->gray = temp2gray(s->rawtemp, s->height, s->width, ref->min_rawtemp, ref->max_rawtemp);
    }
}

void clearStatusImgs(im_status *s)
{
    if (s->rawtemp) {
        delete[] s->rawtemp;
        s->rawtemp = NULL;
    }
    s->gray.release();
}

void initStatus(im_status *s, img_stream *is, vector<poi> POI)
{

    setStatusImgs(s, is);

    for (int i = 0; i < s->height * s->width; i++) {
        s->min_rawtemp = (s->min_rawtemp > s->rawtemp[i]) ? s->rawtemp[i] : s->min_rawtemp;
        s->max_rawtemp = (s->max_rawtemp < s->rawtemp[i]) ? s->rawtemp[i] : s->max_rawtemp;
    }

    s->POI = POI;
}

void calcCurrStatus(im_status *curr, im_status *ref, img_stream *is)
{

    setStatusImgs(curr, is, ref);

    // For 1st time run
    if (curr->POI.size() == 0)
        curr->POI = ref->POI;

    curr->POI = ref->POI; // Placeholder until tracking is implemented
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
    if (path.empty())
        return {};
    vector<poi> POI;
    pt::ptree root;
    pt::read_json(path, root);
    for (pt::ptree::value_type &p : root.get_child("POI"))
        POI.push_back({ p.second.get<string>("name"), { p.second.get<float>("x"), p.second.get<float>("y") } });
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
    if (event == CV_EVENT_LBUTTONDOWN) {
        vector<poi> &POI = *((vector<poi> *)(param));
        string name = "Point " + to_string(POI.size());
        // The image is upscaled 2x when displaying POI
        // Thus we need to divide coords by 2 when getting mouse input
        POI.push_back({ name, { (float)x/2, (float)y/2 } });
    }
}

void printPOITemp(Camera *c, im_status *s)
{
    cout << "Temperature of points of interest:\n";
    vector<double> temps = getPOITemp(s->POI, s->rawtemp, c, s->width, s->height);
    for (unsigned i = 0; i < s->POI.size(); i++)
        printf("%s=%.2lf\n", s->POI[i].name.c_str(), temps[i]);
    cout << endl;
}

void showPOIImg(string path){
    vector<poi> POI =  readPOI(path);
    Mat img = readJsonImg(path);
    Mat imdraw = drawPOI(img, POI, NULL, NULL, draw_mode::NUM);
    string title = "POI from " + path;
    imshow(title,imdraw);
    waitKey(0);
    destroyAllWindows();
}

int processNextFrame(img_stream *is, im_status *ref, im_status *curr,
                     string window_name, bool enter_POI, VideoWriter *vw)
{
    calcCurrStatus(curr, ref, is);
    printPOITemp(is->camera, curr);
    Mat img = drawPOI(curr->gray, curr->POI, curr->rawtemp,
                      is->camera, curr_draw_mode);

    if (vw)
        vw->write(curr->gray);

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

    if (sigint_received)
        return 1;

    return 0;
}

void processStream(img_stream *is, im_status *ref, im_status *curr, cmd_arguments *args)

{
    int exit = 0;
    VideoWriter *vw = NULL;
    string window_name = "Thermocam-PCB";

    initStatus(ref, is, readPOI(args->POI_import_path));

    if (gui_available)
        namedWindow(window_name, WINDOW_AUTOSIZE);
    if (gui_available && args->enter_POI)
        setMouseCallback(window_name, onMouse, &ref->POI);
    if (!args->vid_out_path.empty())
        vw = new VideoWriter(args->vid_out_path, CV_FOURCC('H', 'F', 'Y', 'U'),
                             CAM_FPS, Size(ref->width, ref->height), 0);

    while (!exit) {
        auto begin = chrono::steady_clock::now();
        exit = processNextFrame(is, ref, curr, window_name, args->enter_POI, vw);
        auto end = chrono::steady_clock::now();

        double process_time_us = duration_us(end - begin);
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
    case 'd':
        args.display_delay_us = atof(arg) * 1000000;
        break;
    case ARGP_KEY_END:
        if (args.license_dir.empty())
            args.license_dir = ".";
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
    { "license-dir",     'l', "FILE",        0, "Path to directory containing WIC license file." },
    { "record-video",    'r', "FILE",        0, "Record video and store it with entered filename"},
    { "load-video",      'v', "FILE",        0, "Load and process video instead of camera feed"},
    { "delay",           'd', "NUM",         0, "Set delay between each measurement/display in seconds."},
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

    if (!args.show_POI_path.empty() && gui_available) {
        showPOIImg(args.show_POI_path);
        exit(0);
    }

    img_stream is;
    im_status ref, curr;
    initImgStream(&is, args.vid_in_path, args.license_dir);

    processStream(&is, &ref, &curr, &args);

    if (!args.POI_export_path.empty())
        writePOI(ref.POI, curr.gray, args.POI_export_path, true);

    clearStatusImgs(&ref);
    clearStatusImgs(&curr);
    clearImgStream(&is);

    return 0;
}
