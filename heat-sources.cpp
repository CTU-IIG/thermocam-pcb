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


static void normalize_and_convert_to_uchar(Mat &mat_in, Mat &mat_out){
    double min, max;
    minMaxLoc(mat_in, &min, &max);
    mat_in.convertTo(mat_out, CV_8UC1,
                     255.0 / (max - min),
                     255.0 / (1.0 - max / min));
}

vector<HeatSource> heatSources(im_status &s, Mat &laplacian_out, Mat &hsImg_out, Mat &detail_out,
                               Mat &hsAvg_out, cv::Ptr<cv::freetype::FreeType2> ft2)
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

    static Mat hsAvg;
    if (hsAvg.empty())
        hsAvg = hsImg * 0.0; // black image of the same type and size
    double alpha = 0.90;
    hsAvg = alpha * hsAvg + (1-alpha) * hsImg;

    vector<HeatSource> hs(lm.size());
    size_t max_hs = 0;
    for (unsigned i=0; i<lm.size(); i++) {
        hs[i].location = lm[i];
        hs[i].temperature = s.get_temperature(detail.at<double>(lm[i]));
        hs[i].neg_laplacian = laplacian.at<double>(lm[i]);
        if (hs[i].neg_laplacian > hs[max_hs].neg_laplacian)
            max_hs = i;
    }

    hsAvg_out = hsAvg.clone();
    // Make the dark colors more visible
    cv::log(0.001+hsAvg_out, hsAvg_out);
    //cv::sqrt(hsAvg_out, hsAvg_out);

    for (auto [in, out] : {
            make_pair(&detail, &detail_out),
            make_pair(&laplacian, &laplacian_out),
            make_pair(&hsImg, &hsImg_out),
            make_pair(&hsAvg_out, &hsAvg_out),
        }) {
        normalize_and_convert_to_uchar(*in, *out);
        applyColorMap(*out, *out, cv::COLORMAP_INFERNO);
        resize(*out, *out, Size(), 2, 2);
    }

    {
        double min, max;
        stringstream ss;

        minMaxLoc(detail, &min, &max);
        ss << fixed << setprecision(2) << s.get_temperature(max) << "–" << s.get_temperature(min) << "=" <<
            s.get_temperature(max) - s.get_temperature(min) << "°C";
        copyMakeBorder(detail_out, detail_out, 0, 15, 0, 0, BORDER_CONSTANT, Scalar(255, 255, 255));
        ft2->putText(detail_out, ss.str(), Point(5,200), 15, Scalar(0, 0, 0), -1, cv::LINE_AA, false);
    }

    copyMakeBorder(laplacian_out, laplacian_out, 0, 15, 0, 0, BORDER_CONSTANT, Scalar(255, 255, 255));
    ft2->putText(laplacian_out, "max: " + to_string(hs[max_hs].neg_laplacian),
                 Point(5,200), 15, Scalar(0, 0, 0), -1, cv::LINE_AA, false);

    return hs;
}

void display_mat(Mat mat){
    resize(mat, mat, Size(), 4, 4);

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", mat);
}
