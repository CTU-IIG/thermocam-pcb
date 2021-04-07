#include "thermocam-pcb.hpp"
#include "CameraCenter.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
namespace pt = boost::property_tree;

struct img_stream {
    bool is_video;
    Camera *camera = nullptr;
    CameraCenter *cc = nullptr;
    VideoCapture *video = nullptr;
    uint16_t min_rawtemp;
    uint16_t max_rawtemp;
};

inline double pixel2Temp(uint8_t px, double min = RECORD_MIN_C,
                         double max = RECORD_MAX_C)
{
    return ((double)px) / 255 * (max - min) + min;
}

void display_mat(Mat mat){
    resize(mat, mat, Size(), 4, 4);

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", mat);
}

void normalize_and_convert_to_uchar(Mat &mat){
    double min, max;
    minMaxLoc(mat, &min, &max);
    mat -= min;
    mat *= 255.0 / (max - min);
    mat.convertTo(mat, CV_8UC1);
}

// Get local maxima by dilation and comparing with original image
vector<Point> localMaxima(Mat I, Mat &hs)
{
    if (I.empty())
        return {};
    Mat imageLM, localMaxima;
    dilate(I, imageLM, getStructuringElement(MORPH_RECT, cv::Size (3, 3)));

    localMaxima = I >= imageLM;
    hs = localMaxima.clone();

    vector<Point> locations;
    findNonZero(localMaxima, locations);
    return locations;
}

double getTemp(Point p, img_stream *is, im_status *s)
{
    if (p.y < 0 || p.y > s->height || p.x < 0 || p.x > s->width) {
        cerr << "Point at (" << p.x << "," << p.y << ") out of image!" << endl;
        return nan("");
    }
    int idx = s->width * p.y + p.x;

    if (is->is_video)
        return pixel2Temp(s->gray.data[idx]);
    else
        return is->camera->CalculateTemperatureC(s->rawtemp[idx]);
}

vector<poi> heatSources(im_status *s, img_stream *is, vector<Mat> &hs_images)
{
    for (auto &p : s->heat_sources_border) {
        if (p.x < 0 || p.x > s->width || p.y < 0 || p.y > s->height) {
            cerr << "Heat source border out of the image!" << endl;
            return {};
        }
    }

    Mat I = s->gray.clone();
    I.convertTo(I, CV_64F);

    double blur_sigma = 5;
    GaussianBlur(I, I, Size(0, 0), blur_sigma, blur_sigma);
    Laplacian(I, I, I.depth());
    I = -I;
    hs_images[0] = I.clone();

    // Mask points outside of heat source border polygon
    Mat mask(I.rows, I.cols, CV_64F, std::numeric_limits<double>::min());
    Mat border_int32; // fillConvexPoly needs int32 points
    Mat(s->heat_sources_border).convertTo(border_int32, CV_32S);

    fillConvexPoly(mask, border_int32, Scalar(1), CV_AA);
    I = I.mul(mask); // Mask all values outside of border

    // Crop minimum rectangle containing border polygon for speed
    int x_max = INT_MIN, y_max = INT_MIN, x_min = INT_MAX, y_min = INT_MAX;
    for (auto el : s->heat_sources_border) {
        x_max = (x_max < el.x) ? el.x : x_max;
        y_max = (y_max < el.y) ? el.y : y_max;
        x_min = (x_min > el.x) ? el.x : x_min;
        y_min = (y_min > el.y) ? el.y : y_min;
    }

    I = I(Rect(x_min,y_min,x_max-x_min,y_max-y_min));

    /* Applies perspective transform to obtain correct detail of the core */
    vector<Point2f> rect = {Point2f(0, 0), Point2f(x_max-x_min, 0), Point2f(x_max-x_min, y_max-y_min), Point2f(0, y_max-y_min)};
    Mat transform = getPerspectiveTransform(s->heat_sources_border, rect);
    warpPerspective(hs_images[0], hs_images[0], transform, Rect(x_min,y_min,x_max-x_min,y_max-y_min).size());
    prepare_mat_for_display(hs_images[0]);

    vector<Point> lm = localMaxima(I, hs_images[1]);

    resize(hs_images[0], hs_images[0], Size(), 4, 4);
    resize(hs_images[1], hs_images[1], Size(), 4, 4);

    vector<poi> hs(lm.size());
    for (unsigned i=0; i<lm.size(); i++) {
        hs[i].p = lm[i] + Point(x_min,y_min);
        hs[i].temp = getTemp(hs[i].p,is,s);
        hs[i].neg_laplacian = I.at<double>(lm[i]);
    }

    return hs;
}

bool set_border_by_POI_names(string heat_sources_border_file, const vector<poi> &POI, vector<Point2f> &hs_border)
{
    pt::ptree root;
    pt::read_json(heat_sources_border_file, root);

    auto child = root.get_child_optional("names");
    if(! child){return false;}

    // Try to find POI by names
    for (pt::ptree::value_type &name : root.get_child("names")){
        for(poi p : POI){
            if ((string)name.second.data() == p.name){
                hs_border.push_back(p.p);
                break;
            }
        }
    }

    return hs_border.size() == 4;
}
