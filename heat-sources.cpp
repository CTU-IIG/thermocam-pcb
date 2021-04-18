#include "heat-sources.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

static vector<Point> localMaxima(Mat I, Mat &hs)
{
    if (I.empty())
        return {};
    Mat imageLM, localMaxima;
    dilate(I, imageLM, getStructuringElement(MORPH_RECT, cv::Size (3, 3)));

    localMaxima = ((I == imageLM) & (I > 0));

    // mask non-maximum points
    I.copyTo(hs, localMaxima);

    vector<Point> locations;
    findNonZero(localMaxima, locations);
    return locations;
}


static void normalize_and_convert_to_uchar(Mat &mat){
    double min, max;
    minMaxLoc(mat, &min, &max);
    mat -= min;
    mat *= 255.0 / (max - min);
    mat.convertTo(mat, CV_8UC1);
}

vector<HeatSource> heatSources(im_status &s, Mat &laplacian_out, Mat &hsImg_out, Mat &detail_out)
{
    for (auto &p : s.heat_sources_border) {
        if (p.x < 0 || p.x > s.width || p.y < 0 || p.y > s.height) {
            cerr << "Heat source border out of the image!" << endl;
            return {};
        }
    }

    const Size sz(100, 100); // Size of heat sources image
    vector<Point2f> detail_rect = { {0, 0}, {float(sz.width), 0}, {float(sz.width), float(sz.height)}, {0, float(sz.height)} };

    Mat transform = getPerspectiveTransform(s.heat_sources_border, detail_rect);
    Mat raw_float, detail;
    s.rawtemp.convertTo(raw_float, CV_64F);
    warpPerspective(raw_float, detail, transform, sz);

    Mat blur, laplacian, hsImg;
    const double blur_sigma = 7;
    GaussianBlur(detail, blur, Size(0, 0), blur_sigma, blur_sigma);
    Laplacian(blur, laplacian, blur.depth());
    laplacian *= -1;

    vector<Point> lm = localMaxima(laplacian, hsImg);

    vector<HeatSource> hs(lm.size());
    for (unsigned i=0; i<lm.size(); i++) {
        hs[i].location = lm[i];
        hs[i].temperature = s.get_temperature(detail.at<double>(lm[i]));
        hs[i].neg_laplacian = laplacian.at<double>(lm[i]);
    }

    normalize_and_convert_to_uchar(laplacian);
    normalize_and_convert_to_uchar(detail);
    normalize_and_convert_to_uchar(hsImg);
    applyColorMap(laplacian, laplacian, cv::COLORMAP_INFERNO);
    applyColorMap(detail, detail, cv::COLORMAP_INFERNO);
    applyColorMap(hsImg, hsImg, cv::COLORMAP_INFERNO);
    //equalizeHist(detail, detail);   //for display purpose only, nothing is visible otherwise.

    if(!laplacian.empty())
        resize(laplacian, laplacian, Size(), 2, 2);
    if(!hsImg.empty())
        resize(hsImg, hsImg, Size(), 2, 2);
    if(!detail.empty())
        resize(detail, detail, Size(), 2, 2);

    detail_out = detail;
    laplacian_out = laplacian;
    hsImg_out = hsImg;

    return hs;
}

void display_mat(Mat mat){
    resize(mat, mat, Size(), 4, 4);

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", mat);
}
