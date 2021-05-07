#include "thermo_img.hpp"
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

vector<HeatSource> im_status::heatSources(cv::Ptr<cv::freetype::FreeType2> ft2)
{
    for (auto &p : heat_sources_border) {
        if (p.x < 0 || p.x > width() || p.y < 0 || p.y > height()) {
            cerr << "Heat source border out of the image!" << endl;
            return {};
        }
    }

    const Size sz(100, 100); // Size of heat sources image
    vector<Point2f> detail_rect = { {0, 0}, {float(sz.width), 0}, {float(sz.width), float(sz.height)}, {0, float(sz.height)} };

    Mat transform = getPerspectiveTransform(heat_sources_border, detail_rect);
    Mat raw_float, detail;
    rawtemp.convertTo(raw_float, CV_64F);
    warpPerspective(raw_float, detail, transform, sz);

    Mat blur, laplacian, hsImg;
    const double blur_sigma = 7;
    GaussianBlur(detail, blur, Size(0, 0), blur_sigma, blur_sigma);
    Laplacian(blur, laplacian, blur.depth());
    laplacian *= -1;

    vector<Point> lm = localMaxima(laplacian, hsImg);

    static array<Mat, 3> hsAvg;
    for (auto [i, alpha] : { make_pair(0U, 0.9), {1, 0.99}, {2, 0.999} }) {
        if (hsAvg[i].empty())
            hsAvg[i] = hsImg * 0.0; // black image of the same type and size
        hsAvg[i] = alpha * hsAvg[i] + (1-alpha) * hsImg;

        hsAvg_out[i] = hsAvg[i].clone();
        // Make the dark colors more visible
        cv::log(0.001+hsAvg_out[i], hsAvg_out[i]);
        //cv::sqrt(hsAvg_out, hsAvg_out);
    }

    vector<HeatSource> hs(lm.size());
    size_t max_hs = 0;
    for (unsigned i=0; i<lm.size(); i++) {
        hs[i].location = lm[i];
        hs[i].temperature = get_temperature(detail.at<double>(lm[i]));
        hs[i].neg_laplacian = laplacian.at<double>(lm[i]);
        if (hs[i].neg_laplacian > hs[max_hs].neg_laplacian)
            max_hs = i;
    }

    for (auto [in, out] : {make_pair(&detail, &detail_out),
                           {&laplacian, &laplacian_out},
                           {&hsImg, &hsImg_out},
                           {&hsAvg_out[0], &hsAvg_out[0]},
                           {&hsAvg_out[1], &hsAvg_out[1]},
                           {&hsAvg_out[2], &hsAvg_out[2]},
        }) {
        normalize_and_convert_to_uchar(*in, *out);
        applyColorMap(*out, *out, cv::COLORMAP_INFERNO);
        resize(*out, *out, Size(), 2, 2);
    }

    {
        double min, max;
        stringstream ss;

        minMaxLoc(detail, &min, &max);
        ss << fixed << setprecision(2) << get_temperature(max) << "–" << get_temperature(min) << "=" <<
            get_temperature(max) - get_temperature(min) << "°C";
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
