#include "img_stream.hpp"
#include "thermo_img.hpp"
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
        return is->camera->calculateTemperatureC(s->rawtemp[idx]);
}

Point2f apply_transformation_on_point(const Matx33f &mat, Point2f p){
    Point3f point = Point3f(p.x, p.y, 1);

    point = mat * point;
    return Point2f(max(0.0f, round(point.x)), max(0.0f, round(point.y)));   //sometimes transformation moves point around 0 close to -1.
}

vector<poi> heatSources(im_status *s, img_stream *is, Mat &laplacian, Mat &hsImg, Mat &detail)
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
    laplacian = I.clone();

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
    auto rect = Rect(x_min,y_min,x_max-x_min,y_max-y_min);
    I = I(rect);

    /* Applies perspective transform to obtain correct detail of the core */
    Mat transform = getPerspectiveTransform(s->heat_sources_border, s->border_frame);
    warpPerspective(laplacian, laplacian, transform, Size(s->border_frame[2]));
    warpPerspective(s->gray, detail, transform, Size(s->border_frame[2]));

    Mat dump;
    vector<Point> lm = localMaxima(I, dump);    //Local maxima has to be calculated on shifted image to gain precise temperatures
    localMaxima(laplacian, hsImg);      //For display we use transformed image

    vector<poi> hs(lm.size());
    for (unsigned i=0; i<lm.size(); i++) {
        hs[i].p = lm[i] + Point(x_min,y_min);
        hs[i].temp = getTemp(hs[i].p,is,s);
        hs[i].neg_laplacian = I.at<double>(lm[i]);

        hs[i].p = apply_transformation_on_point(transform, hs[i].p);
    }

    normalize_and_convert_to_uchar(laplacian);
    equalizeHist(detail, detail);   //for display purpose only, nothing is visible otherwise.

    if(!laplacian.empty())
        resize(laplacian, laplacian, Size(), 4, 4);
    if(!hsImg.empty())
        resize(hsImg, hsImg, Size(), 4, 4);
    if(!detail.empty())
        resize(detail, detail, Size(), 4, 4);

    return hs;
}
