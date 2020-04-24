#include <iostream>
#include <fstream>
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

class Method {
public:
    enum Type {KP_FAST, KP_BRISK, DESC_BRISK};
    Type type;
    std::string name;
    unsigned n; // Number of params
    std::vector<double> params;

    Method(Type type, int n, std::vector<double> p_min,
           std::vector<double> p_max, std::vector<double> step)
    : type(type), n(n), params(p_min), p_min(p_min), p_max(p_max), step(step) {}
    virtual void init() = 0;
    virtual void setParams(std::vector<double> p) = 0;
    void incrementParams();
    void printParams();
    void generate(cv::Mat img, std::vector<cv::KeyPoint> kp_in);
    virtual void calc(cv::Mat img, std::vector<cv::KeyPoint>) = 0;
    virtual void serialize(std::ostream &os) = 0;
    virtual void deserialize(std::istream &is) = 0;
    void load()
    {
        std::string filename = name + ".dat";
        std::ifstream ifs(filename, std::ifstream::in);
        deserialize(ifs);
        ifs.close();
    }
    void store()
    {
        std::string filename = name + ".dat";
        std::ofstream ofs(filename, std::ofstream::out);
        serialize(ofs);
        ofs.close();
    }

protected:
    cv::Ptr<cv::Feature2D> f2d;
    std::vector<double> p_min; // Parameter min value
    std::vector<double> p_max; // Parameter max value
    std::vector<double> step;
};

void Method::printParams()
{
    std::cout << "Params: ";
    for (unsigned i = 0; i < n; i++)
        std::cout << params[i] << ", ";
    std::cout << std::endl;
}

void Method::incrementParams() 
{
    params[0] += step[0];
    for (unsigned i = 0; i < n - 1; i++) {
        if (params[i] > p_max[i]) {
            params[i] = p_min[i];
            params[i + 1] += step[i + 1];
        }
    }
}

void Method::generate(cv::Mat img, std::vector<cv::KeyPoint> kp_in = {})
{
    setParams(p_min);
    while (1) {
        calc(img, kp_in);
        incrementParams();
        if (params.back() > p_max.back())
            break;
        setParams(params);
    }
}

class KeyPointMethod : public Method {
    public:
        std::vector<std::vector<cv::KeyPoint>> kp;
        KeyPointMethod(Type type, int n, std::vector<double> p_min,
                       std::vector<double> p_max, std::vector<double> step)
            : Method(type, n, p_min, p_max, step)
            {
                init();
            }
        void init()
        {
            switch(type) {
                case Type::KP_FAST:
                    f2d = cv::FastFeatureDetector::create(p_min[0], p_min[1]);
                    name = "kp_fast";
                    break;
                case Type::KP_BRISK:
                    f2d = cv::BRISK::create(p_min[0], p_min[1]);
                    name = "kp_brisk";
                    break;
                default:
                    err(1,"Unknown KeyPointMethod type");
                    break;
            }
        }
        void setParams(std::vector<double> p)
        {
            switch(type) {
                case Type::KP_FAST:{
                    dynamic_cast<cv::FastFeatureDetector*>(f2d.get())->setThreshold(p[0]);
                    dynamic_cast<cv::FastFeatureDetector*>(f2d.get())->setNonmaxSuppression(p[1]);
                    dynamic_cast<cv::FastFeatureDetector*>(f2d.get())->setType(static_cast<cv::FastFeatureDetector::DetectorType>(p[2]));
                    break;
                }
                case Type::KP_BRISK:
                    delete f2d;
                    f2d = cv::BRISK::create(p[0], p[1]);
                    break;
                default:
                    err(1,"Unknown KeyPointMethod type");
                    break;
            }

        }
        void calc(cv::Mat img, std::vector<cv::KeyPoint> kp_in) {
            kp.push_back({});
            f2d->detect(img, kp.back());
        }
        void serialize(std::ostream &os)
        {
            size_t kpsize = kp.size();
            os.write((char *)&kpsize, sizeof(kpsize));
            for (unsigned i=0; i<kpsize; i++) {
                size_t vsize = kp[i].size();
                os.write((char *)&vsize, sizeof(vsize));
                os.write((char *)&kp[i][0], vsize * sizeof(cv::KeyPoint));
            }
        }
        void deserialize(std::istream &is)
        {
            kp.clear();
            size_t kpsize;
            is.read((char *)&kpsize, sizeof(kpsize));
            for (unsigned i = 0; i < kpsize; i++) {
                size_t vsize;
                is.read((char *)&vsize, sizeof(vsize));
                kp[i].resize(vsize);
                is.read((char *)&kp[i][0], vsize * sizeof(cv::KeyPoint));
            }
        }
};


class DescriptorMethod : public Method {
    public:
        std::vector<cv::Mat> desc;
        DescriptorMethod(Type type, int n, std::vector<double> p_min,
                       std::vector<double> p_max, std::vector<double> step)
            : Method(type, n, p_min, p_max, step)
            {
                init();
            }
        void init() {
            switch(type) {
                case Type::DESC_BRISK:
                    f2d = cv::BRISK::create(0,0,p_min[0]);
                    name = "desc_brisk";
                    break;
                default:
                    err(1,"Unknown DescriptorMethod type");
                    break;
            }
        }
        void calc(cv::Mat img, std::vector<cv::KeyPoint> kp_in) {
            desc.push_back(cv::Mat());
            f2d->compute(img, kp_in, desc.back());
            desc.back().convertTo(desc.back(),CV_32F);
        }
        void setParams(std::vector<double> p)
        {
            switch(type) {
                case Type::DESC_BRISK:
                    f2d = cv::BRISK::create(0,0,p[0]);
                    name = "desc_brisk";
                    break;
                default:
                    err(1,"Unknown DescriptorMethod type");
                    break;
            }
        }
        void serialize(std::ostream &os)
        {
            size_t dsize = desc.size();
            os.write((char *)&dsize, sizeof(dsize));
            for (unsigned i=0; i<dsize; i++) {
                int type = desc[i].type();
                os.write((const char*)(&desc[i].rows), sizeof(int));
                os.write((const char*)(&desc[i].cols), sizeof(int));
                os.write((const char*)(&type), sizeof(int));
                os.write((const char*)(desc[i].data), desc[i].elemSize() * desc[i].total());
            }
        }
        void deserialize(std::istream &is)
        {
            size_t dsize;
            is.read((char *)&dsize, sizeof(dsize));
            for (unsigned i=0; i<dsize; i++) {
                int rows, cols, type;
                is.read((char*)(&rows), sizeof(int));
                is.read((char*)(&cols), sizeof(int));
                is.read((char*)(&type), sizeof(int));

                desc[i].create(rows, cols, type);
                is.read((char*)(desc[i].data), desc[i].elemSize() * desc[i].total());
            }
        }
};

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

double evaluateMatching(cv::FlannBasedMatcher *m, cv::Mat d0, cv::Mat d1)
{
    std::vector<std::vector<cv::DMatch>> matches;
    m->knnMatch(d0, d1, matches, 2);
    double thresh = 0.8;

    double dist = 0;
    double good_matches = 0;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < thresh * matches[i][1].distance) {
            dist += matches[i][0].distance;
            good_matches++;
        }
    }      

    return dist/good_matches;
}

// TODO: Implement descriptor map
// TODO: Implement storing descriptor map or something like that - save all keypoints for keypoint method in form: "KP_BRISK_DESC_ORB"
// TODO: Implement loading descriptor map 

int main()
{
    srand (time(NULL));
    std::vector<cv::Mat> imgs(2);
    imgs[0] = cv::imread("../images/imx8_0.png", cv::IMREAD_GRAYSCALE);
    imgs[1] = cv::imread("../images/imx8_1.png", cv::IMREAD_GRAYSCALE);
    init(imgs[0].cols, imgs[0].rows);
    std::vector<cv::Mat> H = generateHomographies(10);
    std::vector<cv::Mat> I = generateTransformedImgs(H, imgs);
    
    KeyPointMethod kp_fast(Method::Type::KP_FAST, 3,{1,0,0},{200,1,2},{1,1,1});
    KeyPointMethod kp_brisk(Method::Type::KP_BRISK,2,{1,0},{200,8},{1,1});

    DescriptorMethod desc_brisk(Method::Type::DESC_BRISK,1,{1},{1},{0.1});

    kp_fast.generate(imgs[0],{});

    desc_brisk.generate(imgs[0],kp_fast.kp[1]);

    for (int i=0; i<kp_fast.kp.size(); i++) {
        cv::Mat draw;
        cv::drawKeypoints(imgs[0], kp_fast.kp[i], draw);
        cv::imshow("Keypoints", draw);
        cv::waitKey(0); 
    }
     
    return 0;
}
