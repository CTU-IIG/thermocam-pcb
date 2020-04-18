#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <err.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>

struct h_params {
    double angle, tx, ty; // Euclidean (translation + rotation)
    double sx, sy;        // Scale
    double shx, shy;      // Shear
    double p1, p2;        // Projective
};

struct method {
    std::string name;
    unsigned n; // Number of params
    std::vector<double> p_min; // Parameter min value
    std::vector<double> p_max; // Parameter max value
    std::vector<double> step;
};

struct kpdesc {
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
};

cv::Mat C, iC; // Centering transformation and inverse
std::vector<cv::Point2d> test_points; // Points for testing homography accuracy
h_params h_min, h_max; // Minimum/maximum parameters of homography

void init(int img_width, int img_height)
{
    double w = (double)img_width;
    double h = (double)img_height;

    // Init centering matrix and inverse
    C =  (cv::Mat_<double>(3, 3) << 1, 0,  w / 2, 0, 1,  h / 2, 0, 0, 1);
    iC = (cv::Mat_<double>(3, 3) << 1, 0, -w / 2, 0, 1, -h / 2, 0, 0, 1);

    // Init point grid for testing homography accuracy
    int grid_p = 5;
    double step_x = w/grid_p;
    double step_y = h/grid_p;
    for (int i = 0; i < grid_p; i++)
        for (int j = 0; j < grid_p; j++)
            test_points.push_back({ i * step_x, j * step_y });

    // Init minimum and maximum homography parameters
    bool H_wild = false; // Fairly wide homography parameter range
    if (H_wild) {
        h_min = { -M_PI, -0.25 * w, -0.25 * h, 
                   0.75,  0.75, -0.25, -0.25, -0.001, -0.001 };
        h_max = {  M_PI,  0.25 * w,  0.25 * h, 
                   1.25,  1.25,  0.25,  0.25,  0.001,  0.001 };
    } else {
        h_min = { -M_PI/8, -0.125 * w, -0.125 * h, 
                   0.9,  0.9, -0.1, -0.1, -0.0001, -0.0001 };
        h_max = {  M_PI/8,  0.125 * w,  0.125 * h, 
                   1.1,  1.1,  0.1,  0.1,  0.0001,  0.0001 };
    }
}

double randd(double min, double max)
{
    double d = (double)rand() / RAND_MAX;
    return min + d * (max - min);
}

h_params randomHomographyParams(h_params *min, h_params *max)
{
    h_params h;
    h.angle = randd(min->angle, max->angle);
    h.tx = randd(min->tx, max->tx);
    h.ty = randd(min->ty, max->ty);
    h.sx = randd(min->sx, max->sx);
    h.sy = randd(min->sy, max->sy);
    h.shx = randd(min->shx, max->shx);
    h.shy = randd(min->shy, max->shy);
    h.p1 = randd(min->p1, max->p1);
    h.p2 = randd(min->p2, max->p2);
    return h;
}

cv::Mat getHomography(h_params *h, cv::Mat C = C, cv::Mat iC = iC)
{
    cv::Mat E, S, P; // Euclidean, Shear, Projective transforms
    E = (cv::Mat_<double>(3,3) << 
         cos(h->angle), -sin(h->angle), h->tx,
         sin(h->angle),  cos(h->angle), h->ty,
         0, 0, 1);
    E = C * E * iC; // Rotation around center instead of left upper corner
    S = (cv::Mat_<double>(3,3) << h->sx, h->shx, 0, h->shy, h->sy, 0, 0, 0, 1);
    P = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, h->p1, h->p2, 1);
    return E * S * P;
}

cv::Mat randomHomography(h_params *min = &h_min, h_params *max = &h_max)
{
    h_params h = randomHomographyParams(min, max);
    return getHomography(&h);
}

void showTransformError(cv::Mat img, cv::Mat H_ref, cv::Mat H,
                        std::vector<cv::Point2d> p)
{
    cv::Mat img_ref, img_transform;
    std::vector<cv::Point2d> pref, pH;
    cv::warpPerspective(img, img_ref, H_ref, img.size());
    perspectiveTransform(p, pref, H_ref);
    perspectiveTransform(p, pH, H);
    for (unsigned i = 0; i < p.size(); i++)
        cv::line(img_ref, pref[i], pH[i], cv::Scalar(0, 0, 255), 2);
    cv::imshow("Perspective transform errors", img_ref);
    cv::waitKey(0);
}

double meanTransformError(cv::Mat H, cv::Mat Href, std::vector<cv::Point2d> p)
{
    std::vector<cv::Point2d> pref, pH;
    double err = 0;
    perspectiveTransform(p, pref, Href);
    perspectiveTransform(p, pH, H);
    for (unsigned i = 0; i < p.size(); i++) {
        cv::Point2d diff = pref[i] - pH[i];
        err += sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    return err/p.size();
}

std::vector<cv::Mat> generateHomographies(int count)
{
    std::vector<cv::Mat> H(count);
    for (int i = 0; i < count; i++)
        H[i] = randomHomography();
    return H;
}

std::vector<cv::Mat> generateTransformedImgs(std::vector<cv::Mat> H,
                                             std::vector<cv::Mat> imgs)
{
    std::vector<cv::Mat> I(H.size());
    int img_idx = 0;
    for (unsigned i = 0; i < H.size(); i++) {
        cv::warpPerspective(imgs[img_idx], I[i], H[i], imgs[img_idx].size());
        img_idx = (img_idx + 1) % imgs.size();
    }
    return I;
}

void updateMethodParams(std::vector<double> &params, method *m) 
{
    params[0] += m->step[0];
    for (unsigned i = 0; i < m->n - 1; i++) {
        if (params[i] > m->p_max[i]) {
            params[i] = m->p_min[i];
            params[i + 1] += m->step[i + 1];
        }
    }
}

// We do not need to store all keypoints - methods with chaning parameters may generate the same keypoint many times, and we just need to store the parameters which generated it
std::vector<std::vector<cv::KeyPoint>> generateKeypoints(method *m,
                                                         cv::Mat img)
{
    cv::Ptr<cv::Feature2D> f2d;
    std::vector<std::vector<cv::KeyPoint>> kp;

    std::vector<double> params = m->p_min;
    while (1) {
        std::cout << "Params: ";
        for (unsigned i = 0; i < m->n; i++)
            std::cout << params[i] << ", ";
        std::cout << std::endl;

        if(!m->name.compare("FAST"))
            f2d = cv::FastFeatureDetector::create(params[0], params[1]);
        else if (!m->name.compare("BRISK"))
            f2d = cv::BRISK::create(params[0], params[1]);
        else 
            err(1,"Unknown method");

        kp.push_back({});
        f2d->detect(img, kp.back());

        updateMethodParams(params, m);
        if (params[m->n - 1] > m->p_max[m->n - 1])
            break;
    }
    return kp; 
}

std::vector<kpdesc> generateDescriptors(method *m, std::vector<cv::KeyPoint> kp_in, cv::Mat img)
{
    cv::Ptr<cv::Feature2D> f2d;
    std::vector<kpdesc> kpd;

    std::vector<double> params = m->p_min;
    while (1) {
        std::cout << "Params: ";
        for (unsigned i = 0; i < m->n; i++)
            std::cout << params[i] << ", ";
        std::cout << std::endl;

        if(!m->name.compare("BRISK"))
            f2d = cv::BRISK::create(0,0,params[0]);
        else 
            err(1,"Unknown method");
        kpd.push_back({kp_in,cv::Mat()});
        f2d->compute(img, kpd.back().kp, kpd.back().desc);

        updateMethodParams(params, m);
        if (params[m->n - 1] > m->p_max[m->n - 1])
            break;
    }
    return kpd;
}

int main()
{
    srand (time(NULL));
    std::vector<cv::Mat> imgs(2);
    imgs[0] = cv::imread("../../images/imx8_0.png", cv::IMREAD_GRAYSCALE);
    imgs[1] = cv::imread("../../images/imx8_1.png", cv::IMREAD_GRAYSCALE);
    init(imgs[0].cols, imgs[0].rows);
    //std::vector<cv::Mat> H = generateHomographies(10);
    //std::vector<cv::Mat> I = generateTransformedImgs(H, imgs);
    
    method kp_fast =  {"FAST", 2,{1,0},{200,1},{1,1}};
    method kp_brisk = {"BRISK",2,{1,0},{200,8},{1,1}};

    method desc_brisk = {"BRISK",1,{0.5},{2},{0.1}};

    std::vector<std::vector<cv::KeyPoint>> kp = generateKeypoints(&kp_fast,imgs[0]);
   
    std::vector<kpdesc> kpd = generateDescriptors(&desc_brisk, kp[0], imgs[0]);
    /* 
    for (int i=0; i<kp.size(); i++) {
        cv::Mat draw;
        cv::drawKeypoints(imgs[0], kp[i], draw);
        cv::imshow("Keypoints", draw);
        cv::waitKey(0); 
    }*/
    return 0;
}
