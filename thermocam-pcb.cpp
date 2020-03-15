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

#define DEBUG 0

using namespace cv;
namespace pt = boost::property_tree;

#define CAM_FPS 9 // The camera is 9Hz

/* Command line options */
bool enter_POI = false;
string POI_export_path;
string POI_import_path;
string show_POI_path;
string license_dir;
string vid_in_path;
string vid_out_path;
int display_delay_us = 0;

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

void detectDisplay()
{
    gui_available = getenv("DISPLAY") != NULL;
    if (!gui_available)
        cerr << "\"DISPLAY\" environment variable not set! All graphical functions(displaying, entering points, recording video) turned off.\n";
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

void drawPOI(Mat &A, vector<poi> POI, uint16_t *curr_rawtemp, Camera *c, draw_mode mode,
             Scalar color = Scalar(0, 0, 255))
{

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
}

void setStatusHeightWidth(im_status *s, Camera *camera, VideoCapture *video)
{
    if (video) {
        s->height = video->get(CV_CAP_PROP_FRAME_HEIGHT);
        s->width = video->get(CV_CAP_PROP_FRAME_WIDTH);
    } else {
        s->height = camera->GetSettings()->GetResolutionY();
        s->width = camera->GetSettings()->GetResolutionX();
    }
}

void setStatusImgs(im_status *s, Camera *camera, VideoCapture *video, im_status *ref = NULL)
{

    if (!s->height || !s->width)
        setStatusHeightWidth(&(*s), camera, video);

    if (s->rawtemp == NULL)
        s->rawtemp = new uint16_t[s->height * s->width]{ 0 };

    if (video) {
        *video >> s->gray;
        if (s->gray.empty()) {
            video->set(CV_CAP_PROP_POS_AVI_RATIO, 0);
            *video >> s->gray;
        }
        cvtColor(s->gray, s->gray, COLOR_RGB2GRAY);
    } else {
        uint16_t *tmp = (uint16_t *)camera->RetreiveBuffer();
        memcpy(s->rawtemp, tmp, s->height * s->width * sizeof(uint16_t));
        camera->ReleaseBuffer();
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

void initStatus(im_status *s, Camera *camera, vector<poi> POI, VideoCapture *video = NULL)
{

    setStatusImgs(s,camera,video);

    for (int i = 0; i < s->height * s->width; i++) {
        s->min_rawtemp = (s->min_rawtemp > s->rawtemp[i]) ? s->rawtemp[i] : s->min_rawtemp;
        s->max_rawtemp = (s->max_rawtemp < s->rawtemp[i]) ? s->rawtemp[i] : s->max_rawtemp;
    }

    s->POI = POI;
}

void calcCurrStatus(im_status *curr, im_status *ref, Camera *camera, VideoCapture *video = NULL)
{

    setStatusImgs(curr,camera,video,ref);

    // For 1st time run
    if (curr->POI.size() == 0)
        curr->POI = ref->POI;

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

void writePOI(vector<poi> POI, Mat last_img, string path)
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
}

vector<poi> readPOI(string path)
{
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

void inputPOI(Camera* camera, vector<poi>* POI, VideoCapture* video, Mat& last_img){

    cout << "\nClick on the image to enter points.\nPress Esc to delete the last entered point.\nPress Enter to finish selection and store points.\nPress Tab to change between different views." << endl;
    
    namedWindow("Selecting points of interest", WINDOW_AUTOSIZE );
    setMouseCallback("Selecting points of interest", onMouse, POI);

    im_status s;
    while(1){
        char key = (waitKey(1000/CAM_FPS) & 0xEFFFFF);
        if (key == 27 && POI->size() > 0) // Esc
            POI->pop_back();
        if (key == 10 || key == 13) // Enter
            break; 
        if (key == 9) // Tab
            curr_draw_mode = next(curr_draw_mode);
        initStatus(&s,camera,{},video);
        Mat img;
        cvtColor(s.gray,img,COLOR_GRAY2RGB);
        drawPOI(img, *POI, s.rawtemp, camera, curr_draw_mode);
        imshow( "Selecting points of interest", img);
    }
    last_img = s.gray.clone();
    clearStatusImgs(&s);
    destroyWindow("Selecting points of interest");
}

void printPOITemp(Camera *c, vector<poi> POI, im_status *s)
{
    cout << "Temperature of points of interest:\n";
    vector<double> temps = getPOITemp(POI, s->rawtemp, c, s->width, s->height);
    for (unsigned i = 0; i < POI.size(); i++)
        printf("%s=%.2lf\n", POI[i].name.c_str(), temps[i]);
    cout << endl;
}

void recordVideo(Camera *camera, string vid_out_path)
{

    im_status ref, curr;
    int height = camera->GetSettings()->GetResolutionY();
    int width = camera->GetSettings()->GetResolutionX();
    // Save lossless video to not introduce artifacts for later image processing
    VideoWriter video(vid_out_path, CV_FOURCC('F', 'F', 'V', '1'), CAM_FPS, Size(width, height));

    // Wait to start the recording
    while (1) {
        char key = (waitKey(1000 / CAM_FPS) & 0xEFFFFF);
        if (key == 10 || key == 13) // Enter
            break;
        initStatus(&ref, camera, {}, {});
        imshow("Press Enter to start recording", ref.gray);
    }
    destroyAllWindows();

    // Recording
    while (1) {
        char key = (waitKey(1000 / CAM_FPS) & 0xEFFFFF);
        if (key == 27) // Esc
            break;
        calcCurrStatus(&curr, &ref, camera);
        Mat M;
        cvtColor(curr.gray, M, COLOR_GRAY2RGB); // Saving only works on color video
        video.write(M);
        imshow("Press Esc to stop recording and save video", curr.gray);
    }
    destroyAllWindows();

    video.release();

    clearStatusImgs(&ref);
    clearStatusImgs(&curr);
}

void showPOIImg(string path){
    vector<poi> POI =  readPOI(path);
    Mat img = readJsonImg(path);
    while(1) {
        Mat imdraw;
        cvtColor(img, imdraw, COLOR_GRAY2RGB);
        drawPOI(imdraw, POI, NULL, NULL, curr_draw_mode);
        string title = "Showing POI from " + path + " - Press Esc to exit";
        imshow(title,imdraw);
        char key = waitKey(0) & 0xEFFFFF;
        if(key == 27) // Esc
            break;
        if(key == 9) // Tab
            curr_draw_mode = next(curr_draw_mode);
    }
    destroyAllWindows();
}

static error_t parse_opt(int key, char *arg, struct argp_state *argp_state)
{
    switch (key) {
    case 'e':
        enter_POI = true;
        if(arg != NULL)
            POI_export_path = arg;
        break;
    case 'p':
        POI_import_path = arg;
        break;
    case 's':
        show_POI_path = arg;
        break;
    case 'l':
        license_dir = arg;
        break;
    case 'r':
        vid_out_path = arg;
        break;
    case 'v':
        vid_in_path = arg;
        break;
    case 'd':
        display_delay_us = atof(arg) * 1000000;
        break;
    case ARGP_KEY_END:
        if (license_dir.empty())
            license_dir = ".";
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

    "Requires path to directory containing WIC license file to run."
};

int main(int argc, char **argv)
{

    argp_parse(&argp, argc, argv, 0, 0, NULL);

    signal(SIGINT, signalHandler);
    detectDisplay();

    CameraCenter* cc = NULL;
    Camera* camera = NULL;
    VideoCapture* video = NULL;

    if (!show_POI_path.empty() && gui_available) {
        showPOIImg(show_POI_path);
        exit(0);
    }

    if (!vid_in_path.empty()) {
        video = new VideoCapture(vid_in_path);
    } else {
        initCamera(license_dir, cc, camera);
        camera->StartAcquisition();
    }

    // TODO: Set camera FPS from camera itself
    if(!vid_out_path.empty() && camera && gui_available)
        recordVideo(camera,vid_out_path);

    vector<poi> POI;

    if (!POI_import_path.empty())
        POI = readPOI(POI_import_path);
    if (enter_POI && gui_available) {
        Mat last_img;
        inputPOI(camera, &POI, video, last_img);
        if (!POI_export_path.empty()) {
            writePOI(POI, last_img, POI_export_path);
            cout << "Points saved to " << POI_export_path << endl;
        }
    }

    im_status ref, curr;
    initStatus(&ref, camera, POI, video);

    if (gui_available)
        namedWindow("Thermocam-PCB (Press Esc to exit)", WINDOW_NORMAL);

    while (1) {
        chrono::steady_clock::time_point begin = chrono::steady_clock::now();
        calcCurrStatus(&curr, &ref, camera, video);
        printPOITemp(camera, POI, &curr);
        Mat img;
        cvtColor(curr.gray, img, COLOR_GRAY2RGB);
        drawPOI(img, curr.POI, curr.rawtemp, camera, curr_draw_mode);
        if (gui_available)
            imshow("Thermocam-PCB (Press Esc to exit)", img);

        if (gui_available) {
            char key = waitKey(1) & 0xEFFFFF;
            if (key == 27) // Esc
                break;
            if (key == 9) // Tab
                curr_draw_mode = next(curr_draw_mode);
        }

        if (sigint_received)
            break;

        chrono::steady_clock::time_point end = chrono::steady_clock::now();
        double process_time_us = chrono::duration_cast<chrono::microseconds>(end - begin).count();
        if (display_delay_us > process_time_us)
            usleep(display_delay_us - process_time_us);
    }

    clearStatusImgs(&ref);
    clearStatusImgs(&curr);

    if (camera) {
        camera->StopAcquisition();
        camera->Disconnect();
        delete cc;
    }

    if(video)
        delete video;

    return 0;
}
