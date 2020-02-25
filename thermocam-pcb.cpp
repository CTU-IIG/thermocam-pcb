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

#define DISPLAY_DELAY_MS 112 // The camera is 9Hz, ~111 ms

/* Command line options */
bool enter_POI = false;
string POI_path;
string license_dir;

struct im_status{
    uint16_t* rawtemp;
    uint16_t min_rawtemp=UINT16_MAX, max_rawtemp=0;
    Mat gray;
    vector<Point2f> POI; // Points of interest 
    vector<Point2f> IAB; // Ignored Area Boundary - no keypoints inside this polygon
    Mat mask;            // Mask for ignored area: 0 - ignore, 255 - don't ignore
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
        int idx = A.cols * round(POI[i].y) + round(POI[i].x);
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

void initStatus(im_status* s, Camera* camera, vector<Point2f> POI, vector<Point2f> IAB){

	int height = camera->GetSettings()->GetResolutionY();
	int width = camera->GetSettings()->GetResolutionX();

    s->rawtemp = getRawTemp(camera);
    for(int i = 0; i < height*width; i++) {
        s->min_rawtemp = (s->min_rawtemp > s->rawtemp[i]) ? s->rawtemp[i] : s->min_rawtemp;
        s->max_rawtemp = (s->max_rawtemp < s->rawtemp[i]) ? s->rawtemp[i] : s->max_rawtemp;
    }    

    s->gray = temp2gray(s->rawtemp,height,width);

    s->POI = POI;
    // s->IAB = IAB;
    // s->mask = calcMask(IAB, height, width);
    // fillKeyDesc(s,s->gray, s->mask); 
}

void calcCurrStatus(im_status *curr, im_status *ref, Camera *camera)
{

    curr->rawtemp = getRawTemp(camera);
    curr->gray = temp2gray(curr->rawtemp, ref->gray.rows, ref->gray.cols, ref->min_rawtemp, ref->max_rawtemp);

    // For 1st time run
    if (curr->POI.size() == 0) {
        curr->POI = ref->POI;
        // curr->IAB = ref->IAB;
        // curr->mask = ref->mask.clone();
    }

    /*
    // Calculate new coordinates of points by homography

    fillKeyDesc(curr,curr->gray, curr->mask);

    Mat H = findHomography(ref->kp, curr->kp, ref->desc, curr->desc);
    if (!H.empty()){
        // findHomography may return an empty matrix if there aren't enough points(4)
        // to estimate a homography from them.
        // In this case, the points stay the same as before.
        perspectiveTransform(ref->POI, curr->POI, H);
        perspectiveTransform(ref->IAB, curr->IAB, H);
    }
    curr->mask = calcMask(curr->IAB, curr->gray.rows, curr->gray.cols);
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

void inputPOI(Camera* camera, vector<Point2f>* POI){

    cout << "\nClick on the image to enter points.\nPress Esc to delete the last entered point.\nPress Enter to finish selection and store points." << endl;
    
	int height = camera->GetSettings()->GetResolutionY();
	int width = camera->GetSettings()->GetResolutionX();

    namedWindow("Selecting points of interest", WINDOW_AUTOSIZE );
    setMouseCallback("Selecting points of interest", onMouse, POI);

    while(1){
        char key = (waitKey(DISPLAY_DELAY_MS) & 0xEFFFFF);
        if (key == 27) // Esc
            POI->pop_back();
        if (key == 10 || key == 13) // Enter
            break; 
        uint16_t* rawtemp = getRawTemp(camera);
        Mat img = temp2gray(rawtemp, height, width);
        cvtColor(img,img,COLOR_GRAY2RGB);
        drawPOI(img, *POI, rawtemp, camera, Scalar(0,0,255));
        imshow( "Selecting points of interest", img);
    }
    destroyWindow("Selecting points of interest");
}

void POIDialog(Camera* camera, vector<Point2f>* POI){
    inputPOI(camera, POI);
    if(yesNoDialog("\nWould you like to save these points into a json config file[Y/n]?")){
        cout << "Enter the path to the config file: ";
        string name;
        cin >> name;
        writePOI(*POI, name);
        cout << "Points saved to " << name << endl;
    }
}

void printPOITemp(Camera* camera, vector<Point2f> POI, uint16_t* rawtemp){

    cout << "Temperature of points of interest:\n";
    for(unsigned i=0; i<POI.size(); i++) {
        int idx = camera->GetSettings()->GetResolutionX() * round(POI[i].y) + round(POI[i].x);
        printf("Point %d: %.2lf [°C]\n", i, camera->CalculateTemperatureC(rawtemp[idx]));
    }
    cout << endl;
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
    case ARGP_KEY_END:
        if (license_dir.empty())
            argp_error(argp_state, "Need path to directory with WIC license file");
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

    Camera* camera = initCamera(license_dir);
    camera->StartAcquisition();

    vector<Point2f> POI;

    if (!POI_path.empty())
        POI = readPOI(POI_path);
    if(enter_POI)
        POIDialog(camera, &POI);

    im_status ref, curr;
    initStatus(&ref, camera, POI, {});
    namedWindow("Thermocam-PCB (Press Esc to exit)", WINDOW_AUTOSIZE);

    while (1) {
        calcCurrStatus(&curr, &ref, camera);
        printPOITemp(camera, POI, curr.rawtemp);
        Mat img = curr.gray.clone();
        cvtColor(img, img, COLOR_GRAY2RGB);
        drawPOI(img, curr.POI, curr.rawtemp, camera, Scalar(0, 0, 255));
        imshow("Thermocam-PCB (Press Esc to exit)", img);
        if ((waitKey(DISPLAY_DELAY_MS) & 0xEFFFFF) == 27) // Esc
            break;
    }

    camera->StopAcquisition();
    camera->Disconnect();
    return 0;
}

///////////////////////////////////////////////////////////////////////
// Functions only used in Homography estimation

/*
Mat calcMask(vector<Point2f> IAB, int height, int width)
{
    if (IAB.size() == 0)
        return Mat();

    Mat mask = Mat::ones(height, width, CV_8U) * 255;

    // fillPoly only accepts 2d Point array as input
    vector<Point> v(IAB.begin(), IAB.end());
    Point *a = &v[0];
    const Point *bb[1] = { &a[0] };
    int npt[] = { (int) IAB.size() };
    fillPoly(mask, bb, npt, 1, Scalar(0), 8);
    
    return mask;
}

void drawIA(Mat img, vector<Point2f> IAB, Scalar color)
{
    Mat roi = img.clone();
    Mat mask = calcMask(IAB, img.rows, img.cols);
    if (mask.empty())
        return;
    roi.setTo(color, mask == 0);
    // Semitransparent polygon
    double alpha = 0.8;
    addWeighted(img, alpha, roi, 1.0 - alpha, 0.0, img);
}

static const Mat default_mat = Mat();
void fillKeyDesc(im_status *s, Mat A, Mat mask = default_mat)
{
    Ptr<BRISK> brisk = BRISK::create(10, 6);
    brisk->detectAndCompute(A, mask, s->kp, s->desc);
}

Mat findHomography(vector<KeyPoint> kp_from, vector<KeyPoint> kp_to, Mat desc_from, Mat desc_to)
{
    // Convert descriptors to float form
    desc_to.convertTo(desc_to, CV_32F);
    desc_from.convertTo(desc_from, CV_32F);

    // FLANN keypoint matching
    vector<vector<DMatch>> matches;
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    matcher->knnMatch(desc_to, desc_from, matches, 2);
    sort(matches.begin(), matches.end());

    if (DEBUG) {
        cout << "from kp: " << kp_from.size() << ", to kp: " << kp_to.size() << endl;
        cout << "matches size: " << matches.size() << endl;
    }

    // Select good matches
    double thresh = 0.9;
    vector<DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < thresh * matches[i][1].distance)
            good_matches.push_back(matches[i][0]);
    }

    if (good_matches.size() == 0)
        return Mat();

    vector<Point2f> toP, fromP;
    for (size_t i = 0; i < good_matches.size(); i++) {
        toP.push_back(kp_to[good_matches[i].queryIdx].pt);
        fromP.push_back(kp_from[good_matches[i].trainIdx].pt);
    }

    if (DEBUG)
        cout << "RANSAC input size:" << toP.size() << endl;
    return findHomography(fromP, toP, RANSAC, 1);
}
*/
