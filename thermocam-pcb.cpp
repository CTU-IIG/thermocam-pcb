#include <argp.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

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
string POI_path;
string license_dir;
string vid_in_path;
string vid_out_path;

struct im_status {
    uint16_t *rawtemp = NULL;
    uint16_t min_rawtemp = UINT16_MAX, max_rawtemp = 0;
    Mat gray;
    vector<Point2f> POI; // Points of interest 
    vector<KeyPoint> kp;
    Mat desc;
};

Camera* initCamera(string license_dir)
{
    // Path to directory containing license file
    CameraCenter *cameras = new CameraCenter(license_dir);

    if (cameras->getCameras().size() == 0) {
        cout << "No camera found!" << endl;
        exit(1);
    }

    Camera* c = cameras->getCameras().at(0);

    if (c->Connect() != 0) {
        cout << "Error connecting camera!" << endl;
        exit(1);
    }

    return c;
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

    uint8_t *gray = (uint8_t *)malloc(h * w);

    // Write gray values
    double step = 255 / (double)(max - min); // Make pix values to 0-255
    for (int i = 0; i < h * w; ++i)
        gray[i] = (temp[i] < min) ? 0 : (temp[i] > max) ? 255 : (temp[i] - min) * step;

    Mat G(h, w, CV_8U, gray);
    return G;
}

void drawPOI(Mat A, vector<Point2f> POI, uint16_t *curr_rawtemp, Camera* c, Scalar color)
{
    for (unsigned i = 0; i < POI.size(); i++) {
        // Dot at POI position
        circle(A, POI[i], 3, color, -1);
        // Text with temperature
        if (round(POI[i].y) < 0 || round(POI[i].y) > A.rows ||
            round(POI[i].x) < 0 || round(POI[i].x) > A.cols){
            cout << "POI out of image!" << endl;
            continue;
        }
        int idx = A.cols * round(POI[i].y) + round(POI[i].x);
        if (!curr_rawtemp[idx]) // Skip points with absolute 0 temperature - video read
            continue;
        double temp = c->CalculateTemperatureC(curr_rawtemp[idx]);
        stringstream stream;
        stream << fixed << setprecision(2) << temp << " [C]";
        putText(A, stream.str(), POI[i], FONT_HERSHEY_PLAIN, 1, color, 2);
    }
}

uint16_t* getRawTemp(Camera* camera){
    uint16_t *rawtemp = (uint16_t *) camera->RetreiveBuffer();
    camera->ReleaseBuffer();
    return rawtemp;
}

void vidSetImStatus(im_status *s, VideoCapture video)
{
    video >> s->gray;
    if (s->gray.empty()) {
        video.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
        video >> s->gray;
    }
    cvtColor(s->gray, s->gray, COLOR_RGB2GRAY);

    if (s->rawtemp == NULL)
        s->rawtemp = new uint16_t[s->gray.cols * s->gray.rows]{ 0 };
}

void initStatus(im_status *s, Camera *camera, vector<Point2f> POI, VideoCapture *video = NULL)
{

    if (video) {
        vidSetImStatus(s, *video);
    } else {
        int height = camera->GetSettings()->GetResolutionY();
        int width = camera->GetSettings()->GetResolutionX();
        s->rawtemp = getRawTemp(camera);
        s->gray = temp2gray(s->rawtemp, height, width);
    }

    for (int i = 0; i < s->gray.cols * s->gray.rows; i++) {
        s->min_rawtemp = (s->min_rawtemp > s->rawtemp[i]) ? s->rawtemp[i] : s->min_rawtemp;
        s->max_rawtemp = (s->max_rawtemp < s->rawtemp[i]) ? s->rawtemp[i] : s->max_rawtemp;
    }

    s->POI = POI;
}

void calcCurrStatus(im_status *curr, im_status *ref, Camera *camera, VideoCapture *video = NULL)
{

    if (video) {
        vidSetImStatus(curr, *video);
    } else {
        curr->rawtemp = getRawTemp(camera);
        curr->gray = temp2gray(curr->rawtemp, ref->gray.rows, ref->gray.cols, ref->min_rawtemp, ref->max_rawtemp);
    }

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

void writePOI(vector<Point2f> POI, string path)
{
    pt::ptree root;
    pt::ptree POI_pt;
    for (unsigned i = 0; i < POI.size(); i++) {
        pt::ptree elem;
        elem.put("x", POI[i].x);
        elem.put("y", POI[i].y);
        POI_pt.push_back(std::make_pair("", elem));
    }
    root.add_child("POI", POI_pt);
    pt::write_json(path, root);
}

vector<Point2f> readPOI(string path)
{
    vector<Point2f> POI;
    pt::ptree root;
    pt::read_json(path, root);
    for (pt::ptree::value_type &p : root.get_child("POI"))
        POI.push_back({ p.second.get<float>("x"), p.second.get<float>("y") });
    return POI;
}

bool yesNoDialog(string s){
    cout << s << endl;
    char key;
    while(1){
        cin >> key;
        if (key == 'Y' || key == 'y')
            return true;
        if (key == 'N' || key == 'n')
            return false;
    }
}

void onMouse(int event, int x, int y, int flags, void *param)
{
    if (event == CV_EVENT_LBUTTONDOWN) {
        vector<Point2f> &POI = *((vector<Point2f> *)(param));
        POI.push_back({ (float) x, (float) y });
    }
}

void inputPOI(Camera* camera, vector<Point2f>* POI, VideoCapture* video){

    cout << "\nClick on the image to enter points.\nPress Esc to delete the last entered point.\nPress Enter to finish selection and store points." << endl;
    
    namedWindow("Selecting points of interest", WINDOW_AUTOSIZE );
    setMouseCallback("Selecting points of interest", onMouse, POI);

    while(1){
        char key = (waitKey(1000/CAM_FPS) & 0xEFFFFF);
        if (key == 27 && POI->size() > 0) // Esc
            POI->pop_back();
        if (key == 10 || key == 13) // Enter
            break; 

        im_status s;
        initStatus(&s,camera,{},video);
        cvtColor(s.gray,s.gray,COLOR_GRAY2RGB);
        drawPOI(s.gray, *POI, s.rawtemp, camera, Scalar(0,0,255));
        imshow( "Selecting points of interest", s.gray);
    }
    destroyWindow("Selecting points of interest");
}

void POIDialog(Camera* camera, vector<Point2f>* POI, VideoCapture* video){
    inputPOI(camera, POI, video);
    if(yesNoDialog("\nWould you like to save these points into a json config file[Y/n]?")){
        cout << "Enter the path to the config file: ";
        string name;
        cin >> name;
        writePOI(*POI, name);
        cout << "Points saved to " << name << endl;
    }
}

void printPOITemp(Camera* camera, vector<Point2f> POI, uint16_t* rawtemp){
    if(!camera)
        return;
    cout << "Temperature of points of interest:\n";
    for(unsigned i=0; i<POI.size(); i++) {
        int idx = camera->GetSettings()->GetResolutionX() * round(POI[i].y) + round(POI[i].x);
        printf("Point %d: %.2lf [°C]\n", i, camera->CalculateTemperatureC(rawtemp[idx]));
    }
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
}

static error_t parse_opt(int key, char *arg, struct argp_state *argp_state)
{
    switch (key) {
    case 'e':
        enter_POI=true;
        break;
    case 'p':
        POI_path = arg;
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
    case ARGP_KEY_END:
        if (license_dir.empty() && vid_in_path.empty())
            argp_error(argp_state, "Need path to directory with WIC license file, or path to input video.");
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

/* The options we understand. */
static struct argp_option options[] = {
    { "enter_poi",       'e', 0,             0, "Enter Points of interest by hand." },
    { "poi_path",        'p', "FILE",        0, "Path to config file containing saved POIs." },
    { "license_dir",     'l', "FILE",        0, "Path to directory containing WIC license file." },
    { "record_video",    'r', "FILE",        0, "Record video and store it with entered filename"},
    { "load_video",      'v', "FILE",        0, "Load and process video instead of camera feed"},
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

    Camera* camera = NULL;
    VideoCapture* video = NULL;

    if (!vid_in_path.empty()) {
        video = new VideoCapture(vid_in_path);
    } else {
        camera = initCamera(license_dir);
        camera->StartAcquisition();
    }

    // TODO: Set camera FPS from camera itself
    if(!vid_out_path.empty() && camera)
        recordVideo(camera,vid_out_path);

    vector<Point2f> POI;

    if (!POI_path.empty())
        POI = readPOI(POI_path);
    if (enter_POI)
        POIDialog(camera, &POI, video);

    im_status ref, curr;
    initStatus(&ref, camera, POI, video);
    namedWindow("Thermocam-PCB (Press Esc to exit)", WINDOW_AUTOSIZE);

    while (1) {
        chrono::steady_clock::time_point begin = chrono::steady_clock::now();
        calcCurrStatus(&curr, &ref, camera, video);
        printPOITemp(camera, POI, curr.rawtemp);
        Mat img = curr.gray.clone();
        cvtColor(img, img, COLOR_GRAY2RGB);
        drawPOI(img, curr.POI, curr.rawtemp, camera, Scalar(0, 0, 255));
        imshow("Thermocam-PCB (Press Esc to exit)", img);

        chrono::steady_clock::time_point end = chrono::steady_clock::now();
        double process_time = chrono::duration_cast<chrono::microseconds>(end - begin).count();
        int wait = (process_time > 1000 / CAM_FPS) ? 1 : 1000 / CAM_FPS - process_time;
        if ((waitKey(wait) & 0xEFFFFF) == 27) // Esc
            break;
    }

    if (camera) {
        camera->StopAcquisition();
        camera->Disconnect();
    }
    return 0;
}
