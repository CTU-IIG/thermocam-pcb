#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "point_tracking.h"

struct h_params {
    double angle, tx, ty; // Euclidean (translation + rotation)
    double sx, sy;        // Scale
    double shx, shy;      // Shear
    double p1, p2;        // Projective
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

// Returns average of inlier errors and average number of outliers
std::vector<double> homTestError(cv::Mat img, int num_tests,
                                 std::vector<cv::Point2d> test_points)
{
    double error = 0;
    double outliers = 0;
    std::vector<cv::KeyPoint> kp_0, kp_1;
    cv::Mat desc_0, desc_1, H_ref, H, img_transformed;

    kp_0 = getKeyPoints(img);
    desc_0 = getDescriptors(img, kp_0);

    for (int i = 0; i < num_tests; i++) {
        H_ref = randomHomography();
        cv::warpPerspective(img, img_transformed, H_ref, img.size());
        kp_1 = getKeyPoints(img_transformed);
        desc_1 = getDescriptors(img_transformed, kp_1);
        H = findHomography(kp_0, kp_1, desc_0, desc_1);
        // showTransformError(img, H_ref, H, test_points);
        double err = (H.empty()) ? 1000 : meanTransformError(H, H_ref, test_points);
        if (err < 20)
            error += err;
        else
            outliers++;
    }
    return { error / num_tests, outliers / num_tests };
}

int main()
{
    srand (time(NULL));
    cv::Mat in = cv::imread("imx8_0.png");
    init(in.cols, in.rows);
    std::vector<double> v = homTestError(in, 100, test_points);
    std::cout << "Mean error: " << v[0] << ", mean outliers: " << v[1] << std::endl;
    return 0;
}
