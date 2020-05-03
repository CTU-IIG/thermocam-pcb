#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <err.h>
#include <unordered_map>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <boost/functional/hash.hpp>

using std::vector;
using std::cout;
using std::endl;
using cv::Mat;
using cv::KeyPoint;

struct h_params {
    double angle, tx, ty; // Euclidean (translation + rotation)
    double sx, sy;        // Scale
    double shx, shy;      // Shear
    double p1, p2;        // Projective
};

Mat C, iC; // Centering transformation and inverse
vector<cv::Point2d> test_points; // Points for testing homography accuracy
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
// Hashable KeyPoint to save in hashmap
class HKeyPoint : public KeyPoint {
    public:
    int kp_param_id = 0;
    HKeyPoint(KeyPoint const& kp) : KeyPoint(kp) {}
    void operator = (const KeyPoint& keypoint_)
    {
      KeyPoint::operator=(keypoint_);
    } 

    bool operator == (const HKeyPoint &kp) const {
        float eps = 0.000001;
        return // For some reason, descriptor calc changes angle
               //abs(angle-kp.angle) < eps) &&
               (class_id == kp.class_id)   &&
               (octave   == kp.octave)     &&
               (abs(pt.x-kp.pt.x)   < eps) &&
               (abs(pt.y-kp.pt.y) < eps)   &&
               (abs(response-kp.response) < eps) &&
               (abs(size-kp.size) < eps);
    }

};

struct HKeyPointHasher
{
  std::size_t operator () (const HKeyPoint &key) const 
  {
    std::size_t seed = 0;
    //boost::hash_combine(seed, boost::hash_value(key.angle));
    boost::hash_combine(seed, boost::hash_value(key.class_id));
    boost::hash_combine(seed, boost::hash_value(key.octave));
    boost::hash_combine(seed, boost::hash_value(key.pt.x));
    boost::hash_combine(seed, boost::hash_value(key.pt.y));
    boost::hash_combine(seed, boost::hash_value(key.response));
    boost::hash_combine(seed, boost::hash_value(key.size));
    return seed;
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const vector<T>& v)
{
    os << "[";
    for (int i = 0; i < v.size(); ++i)
        os << v[i] << ",";
    os << "]";
    return os;
}

std::ostream& operator<<(std::ostream &os, KeyPoint const &v) {
    os << "angle: " << v.angle << ", class_id: " << v.class_id << 
          ", octave: " << v.octave << ", x: " << v.pt.x <<  
          ", y: " << v.pt.y << ", response: " << v.response <<
          ", size: " << v.size;
    return os;
}

std::ostream& operator<<(std::ostream &os, HKeyPoint const &v) {
    os << "angle: " << v.angle << ", class_id: " << v.class_id << 
          ", octave: " << v.octave << ", x: " << v.pt.x <<  
          ", y: " << v.pt.y << ", response: " << v.response <<
          ", size: " << v.size;
    return os;
}

class Method {
public:
    enum Type {KP_FAST, KP_BRISK, DESC_BRISK, UNSET};
    Type type;
    std::string name;
    unsigned n; // Number of params
    vector<double> params;

    Method(Type type, int n, vector<double> p_min,
           vector<double> p_max, vector<double> step)
    : type(type), n(n), params(p_min), p_min(p_min), p_max(p_max), step(step) {}
    virtual void init() = 0;
    virtual void reset() = 0;
    virtual void setParams(vector<double> p) = 0;
    void incrementParams();
    void printParams();
    void generate(vector<Mat> imgs, vector<vector<KeyPoint>> kp_in, bool verbose);
    virtual void calc(vector<Mat> imgs, vector<vector<KeyPoint>> kp_in) = 0;
    virtual void serialize(std::ostream &os) = 0;
    virtual void deserialize(std::istream &is) = 0;
    bool load()
    {
        std::string filename = name + ".dat";
        if (access(filename.c_str(), F_OK) != -1)
            return false;
        std::ifstream ifs(filename, std::ifstream::in);
        deserialize(ifs);
        ifs.close();
        return true;
    }
    void store()
    {
        std::string filename = name + ".dat";
        std::ofstream ofs(filename, std::ofstream::out);
        serialize(ofs);
        ofs.close();
    }
    std::string getTypeName(Type t)
    {
        switch (t) {
            case Type::KP_FAST:
                return "kp_fast";
            case Type::KP_BRISK:
                return "kp_brisk";
            case Type::DESC_BRISK:
                return "desc_brisk";
            default:
                err(1, "Unknown type");
        }
    }

protected:
    cv::Ptr<cv::Feature2D> f2d;
    vector<double> p_min; // Parameter min value
    vector<double> p_max; // Parameter max value
    vector<double> step;
};

void Method::printParams()
{
    cout << name << " params: " << params[0];
    for (unsigned i = 1; i < n; i++)
        cout << ", " << params[i];
    cout << " (max: " << p_max[0];
    for (unsigned i = 1; i < n; i++)
        cout << ", " << p_max[i];
    cout << ")" << endl;
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

void Method::generate(vector<Mat> imgs, vector<vector<KeyPoint>> kp_in = {}, bool verbose = true)
{
    reset();
    params = p_min;
    setParams(p_min);
    while (1) {
        if (verbose)
            printParams();
        calc(imgs, kp_in);
        incrementParams();
        if (params.back() > p_max.back())
            break;
        setParams(params);
    }
}

class KeyPointMethod : public Method {
    public:
        vector<vector<vector<KeyPoint>>> kp;
        vector<vector<KeyPoint>> kp_unique;
        vector<std::unordered_map<HKeyPoint, vector<unsigned>, HKeyPointHasher>> hkps;
        KeyPointMethod(Type type, int n, vector<double> p_min,
                       vector<double> p_max, vector<double> step)
            : Method(type, n, p_min, p_max, step)
            {
                init();
            }
        void init()
        {
            switch(type) {
                case Type::KP_FAST:
                    f2d = cv::FastFeatureDetector::create(p_min[0], p_min[1]);
                    break;
                case Type::KP_BRISK:
                    f2d = cv::BRISK::create(p_min[0], p_min[1]);
                    break;
                default:
                    err(1,"Unknown KeyPointMethod type");
                    break;
            }
            name = getTypeName(type);
        }
        void reset()
        {
            kp.clear();
            kp_unique.clear();
            hkps.clear();
        }
        void setParams(vector<double> p)
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
        void calc(vector<Mat> img, vector<vector<KeyPoint>> kp_in)
        {
            kp.push_back({});
            f2d->detect(img, kp.back());
        }
        void serialize(std::ostream &os)
        {
/*            size_t kpsize = kp.size();
            os.write((char *)&kpsize, sizeof(kpsize));
            for (unsigned i=0; i<kpsize; i++) {
                size_t vsize = kp[i].size();
                os.write((char *)&vsize, sizeof(vsize));
                os.write((char *)&kp[i][0], vsize * sizeof(KeyPoint));
            }
*/        }
        void deserialize(std::istream &is)
        {
/*            kp.clear();
            size_t kpsize;
            is.read((char *)&kpsize, sizeof(kpsize));
            for (unsigned i = 0; i < kpsize; i++) {
                size_t vsize;
                is.read((char *)&vsize, sizeof(vsize));
                kp[i].resize(vsize);
                is.read((char *)&kp[i][0], vsize * sizeof(KeyPoint));
            }
*/        }
        void getUnique()
        {
            unsigned num_imgs = kp[0].size();
            kp_unique.clear();
            kp_unique.resize(num_imgs);
            hkps.resize(num_imgs);
            for (unsigned i = 0; i < kp.size(); i++) { // kp params
                for (unsigned j = 0; j < kp[i].size(); j++) { // images
                    for (unsigned k = 0; k < kp[i][j].size(); k++) { // kp vect
                        HKeyPoint hkp = kp[i][j][k];
                        if (hkps[j].find(hkp) == hkps[j].end()) {
                            hkps[j][hkp] = {i};
                            kp_unique[j].push_back(hkp);
                        } else {
                            hkps[j][hkp].push_back(i);
                        }
                    }
                }
            }
        }
};


class DescriptorMethod : public Method {
    public:
        vector<vector<Mat>> desc;
        vector<vector<vector<Mat>>> full_desc = {};
        std::string desc_name;
        vector<vector<vector<unsigned>>> kp_id = {}; //desc - img - kp_id
        vector<std::unordered_map<HKeyPoint, unsigned, HKeyPointHasher>> kp_map;
        DescriptorMethod(Type type, int n, vector<double> p_min,
                       vector<double> p_max, vector<double> step)
            : Method(type, n, p_min, p_max, step)
            {
                init();
            }
        void init() {
            switch(type) {
                case Type::DESC_BRISK:
                    f2d = cv::BRISK::create(0,0,p_min[0]);
                    break;
                default:
                    err(1,"Unknown DescriptorMethod type");
                    break;
            }
            desc_name = getTypeName(type);
        }
        void reset()
        {
            desc.clear();
            full_desc.clear();
            kp_map.clear();
            kp_id.clear();
        }
        void calc(vector<Mat> img, vector<vector<KeyPoint>> kp_in)
        {
            if(kp_map.size() == 0)
                initKpMap(kp_in);

            desc.push_back({});
            f2d->compute(img, kp_in, desc.back());
            kp_id.push_back(vector<vector<unsigned>>(desc.back().size()));

            for (unsigned i=0; i<desc.back().size(); i++) {
                desc.back()[i].convertTo(desc.back()[i],CV_32F);
                for (KeyPoint keypoint : kp_in[i]) {
                    if(kp_map[i].find(keypoint) == kp_map[i].end())
                        err(1,"BE SPOOKED! KEYPOINT NOT FOUND IN KP_MAP!!");
                    kp_id.back()[i].push_back(kp_map[i][keypoint]);
                }
            }
        }
        void initKpMap(vector<vector<KeyPoint>> kp) {
            kp_map.resize(kp.size());
            for (unsigned i=0; i<kp.size(); i++) {
                for (unsigned j=0; j<kp[i].size(); j++) {
                    HKeyPoint hkp = kp[i][j];
                    kp_map[i][hkp] = j;
                }
            }
        }
        void setParams(vector<double> p)
        {
            switch(type) {
                case Type::DESC_BRISK:
                    f2d = cv::BRISK::create(0,0,p[0]);
                    break;
                default:
                    err(1,"Unknown DescriptorMethod type");
                    break;
            }
        }
        void serialize(std::ostream &os)
        {
/*            size_t dsize = full_desc.size();
            os.write((char *)&dsize, sizeof(dsize));
            for (unsigned i=0; i<dsize; i++) {
                size_t size = full_desc[i].size();
                os.write((char *)&size, sizeof(size));
                for (unsigned j=0; j<size; j++) {
                    int type = full_desc[i][i].type();
                    os.write((const char*)(&full_desc[i][j].rows), sizeof(int));
                    os.write((const char*)(&full_desc[i][j].cols), sizeof(int));
                    os.write((const char*)(&type), sizeof(int));
                    os.write((const char*)(full_desc[i][j].data), full_desc[i][j].elemSize() * full_desc[i][j].total());
                }
            }
*/        }
        void deserialize(std::istream &is)
        {
/*            full_desc.clear();
            size_t dsize;
            is.read((char *)&dsize, sizeof(dsize));
            full_desc.resize(dsize);
            for (unsigned i=0; i<dsize; i++) {
                size_t size;
                is.read((char *)&size, sizeof(size));
                full_desc[i].resize(size);
                for (unsigned j=0; j<size; j++) {
                    int rows, cols, type;
                    is.read((char*)(&rows), sizeof(int));
                    is.read((char*)(&cols), sizeof(int));
                    is.read((char*)(&type), sizeof(int));

                    full_desc[i][j].create(rows, cols, type);
                    is.read((char*)(full_desc[i][j].data), full_desc[i][j].elemSize() * full_desc[i][j].total());
                }
            }
*/        }
        void reconstruct (vector<vector<KeyPoint>> &kp_unique, vector<std::unordered_map<HKeyPoint, vector<unsigned>, HKeyPointHasher>> &hkps, unsigned orig_len)
        {
            // reconstruct from desc_params-imgs-descriptor_mat
            // kp is desc-params-imgs-kp
            // reconstruct to imgs-desc_params-kp_params-descriptor_mat
            cout << "Reconstructing" << endl;
            vector<vector<vector<Mat>>> d(desc[0].size(),vector<vector<Mat>>(desc.size(),vector<Mat>(orig_len)));
            for (unsigned i = 0; i < desc.size(); i++) { // desc_params
                for (unsigned j = 0; j < desc[i].size(); j++) { // imgs
                    cout << "dparam " << i << "/" << desc.size() << ", img " << j << "/" << desc[i].size() << ", rows: " << desc[i][j].rows << endl;
                    for (int k = 0; k < desc[i][j].rows; k++) { // kps
                        HKeyPoint hkp = kp_unique[j][kp_id[i][j][k]];
                        if (hkps[j].find(hkp) == hkps[j].end()){
                            std::cerr << "NOT FOUND!! - " << hkp << endl;
                            continue;
                        }
                        vector<unsigned> kp_idx = hkps[j].at(hkp);
                        for (unsigned id : kp_idx) {
                            if(!d[j][i][id].empty())
                                cv::vconcat(d[j][i][id], desc[i][j].row(k), d[j][i][id]);
                            else
                                d[j][i][id] = desc[i][j].row(k);
                            
                        }
                    }
                }
            }
            full_desc = d;
        }

        void setKpMethodType(Type t)
        {
            kpMethodType = t;
            name = getTypeName(t) + "_" + desc_name;
        }
        void load()
        {
            if (kpMethodType == Type::UNSET)
                err(1, "KeyPoint Method type in not set in descriptor generator. Cannot load/store.");
            Method::load();
        }
        void store()
        {
            if (kpMethodType == Type::UNSET)
                err(1, "KeyPoint Method type in not set in descriptor generator. Cannot load/store.");
            if (full_desc.size() == 0)
                err(1, "full_desc not set, cannot store it.");
            Method::store();
            kpMethodType = Type::UNSET; // To not forget to set before generate
        }

    protected:
        Type kpMethodType = Method::UNSET;
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

Mat getHomography(h_params *h, Mat C = C, Mat iC = iC)
{
    Mat E, S, P; // Euclidean, Shear, Projective transforms
    E = (cv::Mat_<double>(3,3) << 
         cos(h->angle), -sin(h->angle), h->tx,
         sin(h->angle),  cos(h->angle), h->ty,
         0, 0, 1);
    E = C * E * iC; // Rotation around center instead of left upper corner
    S = (cv::Mat_<double>(3,3) << h->sx, h->shx, 0, h->shy, h->sy, 0, 0, 0, 1);
    P = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, h->p1, h->p2, 1);
    return E * S * P;
}

Mat randomHomography(h_params *min = &h_min, h_params *max = &h_max)
{
    h_params h = randomHomographyParams(min, max);
    return getHomography(&h);
}

void showTransformError(Mat img, Mat H_ref, Mat H,
                        vector<cv::Point2d> p)
{
    Mat img_ref, img_transform;
    vector<cv::Point2d> pref, pH;
    cv::warpPerspective(img, img_ref, H_ref, img.size());
    perspectiveTransform(p, pref, H_ref);
    perspectiveTransform(p, pH, H);
    for (unsigned i = 0; i < p.size(); i++)
        cv::line(img_ref, pref[i], pH[i], cv::Scalar(0, 0, 255), 2);
    cv::imshow("Perspective transform errors", img_ref);
    cv::waitKey(0);
}

double meanTransformError(Mat H, Mat Href, vector<cv::Point2d> p)
{
    vector<cv::Point2d> pref, pH;
    double err = 0;
    perspectiveTransform(p, pref, Href);
    perspectiveTransform(p, pH, H);
    for (unsigned i = 0; i < p.size(); i++) {
        cv::Point2d diff = pref[i] - pH[i];
        err += sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    return err/p.size();
}

vector<Mat> generateHomographies(int count)
{
    vector<Mat> H(count);
    for (int i = 0; i < count; i++)
        H[i] = randomHomography();
    return H;
}

// Generates image array, first image is original, rest transformed
vector<vector<Mat>> generateTransformedImgs(vector<Mat> H, vector<Mat> imgs)
{
    if (H.size() % imgs.size() != 0)
        err(1, "Number of homographies isn't divisible by number of images!");
    unsigned H_per_img = H.size() / imgs.size();
    vector<vector<Mat>> I(imgs.size(), vector<Mat>(H_per_img+1));
    for (unsigned i = 0; i < imgs.size(); i++) {
        I[i][0] = imgs[i];
        for (unsigned j = 1; j < H_per_img + 1; j++) {
            cv::warpPerspective(imgs[i], I[i][j], H[i * H_per_img + j - 1], imgs[i].size());
        }
    }
    return I;
}

double evaluateMatching(cv::FlannBasedMatcher *m, Mat d0, Mat d1)
{
    cout << d0.rows << endl;
    cout << d1.rows << endl;
    vector<vector<cv::DMatch>> matches;
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

void testDescriptorMap(vector<vector<vector<Mat>>> &uniq, vector<vector<vector<Mat>>> &brute)
{
    cout << "Unique: " << uniq.size() << ", brute: " << brute.size() << endl; 
    if (uniq.size() != brute.size())
        err(1, "Unique and brute size mismatch, unique: %u");
    for (unsigned i = 0; i < uniq.size(); i++) { // imgs
        cout << "Unique[" << i << "] : " << uniq[i].size() << ", brute[" << i << "]: " << brute[i].size() << endl; 
        if (uniq[i].size() != brute[i].size())
            err(1, "Unique and brute size mismatch, unique: %u", uniq[i].size());
        for (unsigned j = 0; j < uniq[i].size(); j++) { // descs
            cout << "Unique[" << i << "][" << j << "] : " << uniq[i][j].size() << ", brute[" << i << "][" << j << "] : " << brute[i][j].size() << endl; 
            if (uniq[i][j].size() != brute[i][j].size())
                err(1, "Unique and brute size mismatch");
            for (unsigned k = 0; k < uniq[i][j].size(); k++) { // kps
                cout << "Unique[" << i << "][" << j << "][" << k << "] : " << uniq[i][j][k].rows << ", brute[" << i << "][" << j << "][" << k << "] : " << brute[i][j][k].rows << endl; 

                if (uniq[i][j][k].rows != brute[i][j][k].rows)
                    err(1, "Unique and brute size mismatch");
                Mat res;
                cv::sort(uniq[i][j][k],uniq[i][j][k], cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);
                cv::sort(brute[i][j][k],brute[i][j][k], cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);
                cv::compare(uniq[i][j][k],brute[i][j][k], res, cv::CMP_NE);
                if (cv::sum(res)[0])
                    cout << "Diff is: " << cv::sum(res)[0] << endl;
            }
        }
    }
}

void test(vector<vector<Mat>> I, KeyPointMethod *kpm,
          DescriptorMethod *dm)
{
    vector<vector<vector<Mat>>> uniq, brute;
    for (unsigned i = 0; i < I.size(); i++) {
        kpm->generate(I[i], {});
        kpm->getUnique();
        dm->setKpMethodType(kpm->type);
        vector<vector<KeyPoint>> tmp = kpm->kp_unique;
        cout << "Generating from unique kp" << endl;
        dm->generate(I[i], tmp);
        dm->reconstruct(tmp, kpm->hkps, kpm->kp.size());
        uniq = dm->full_desc;
        cout << "Generating from brute kp" << endl;
        dm->generate(I[i], kpm->kp[0]);
        brute.resize(
            I[i].size(),
            vector<vector<Mat>>(
                dm->desc.size(), vector<Mat>(kpm->kp.size())));
        for (unsigned j = 0; j < kpm->kp.size(); j++) {
            dm->generate(I[i], kpm->kp[j]);
            for (unsigned k = 0; k < dm->desc.size(); k++) {
                for (unsigned l = 0; l < dm->desc[k].size(); l++) {
                    brute[l][k][j] = dm->desc[k][l];
                }
            }
        }
        testDescriptorMap(uniq, brute);
    }
}

vector<vector<vector<vector<double>>>> evaluateKpDesc(vector<vector<Mat>> I, KeyPointMethod *kpm, DescriptorMethod *dm)
{
    cv::FlannBasedMatcher m;
    vector<vector<vector<vector<double>>>> score(I.size());
    for (unsigned i=0; i<I.size(); i++) {
        kpm->generate(I[i], {});
        kpm->getUnique();
        dm->setKpMethodType(kpm->type);
        vector<vector<KeyPoint>> tmp = kpm->kp_unique;
        dm->generate(I[i], tmp);
        dm->reconstruct(tmp, kpm->hkps, kpm->kp.size());
        score[i].resize(dm->full_desc.size()-1, vector<vector<double>>(dm->full_desc[0].size(),vector<double>(dm->full_desc[0][0].size())));
        for (unsigned j=1; j<dm->full_desc.size(); j++) // Generated images
            for (unsigned k=0; k<dm->full_desc[j].size(); k++) // desc params
                for (unsigned l=0; l<dm->full_desc[j][k].size(); l++) // kp params
                    score[i][j-1][k][l] = evaluateMatching(&m, dm->full_desc[0][k][l], dm->full_desc[j][k][l]);
    }
    return score; 
}

// TODO: Speed up reconstruct
// TODO: Add source image path to saved kp/desc to save time
// TODO: Measure and save option calc time

//    KeyPointMethod kp_brisk(Method::Type::KP_BRISK,2,{1,0},{200,8},{1,1});

int main()
{
    //srand (time(NULL));
    srand (0);
    vector<Mat> imgs(2);
    imgs[0] = cv::imread("../images/imx8_0.png", cv::IMREAD_GRAYSCALE);
    imgs[1] = cv::imread("../images/imx8_1.png", cv::IMREAD_GRAYSCALE);
    init(imgs[0].cols, imgs[0].rows);
    vector<Mat> H = generateHomographies(4);
    vector<vector<Mat>> I = generateTransformedImgs(H, imgs);

    KeyPointMethod kp_fast(Method::Type::KP_FAST, 3,{1,0,0},{3,1,0},{1,1,1});
    DescriptorMethod desc_brisk(Method::Type::DESC_BRISK,1,{0.8},{0.9},{0.1});

    test(I, &kp_fast, &desc_brisk);
    //evaluateKpDesc(I, &kp_fast, &desc_brisk);


/*
    for (int i=0; i<kp_fast.kp.size(); i++) {
        Mat draw;
        cv::drawKeypoints(imgs[0], kp_fast.kp[i], draw);
        cv::imshow("Keypoints", draw);
        cv::waitKey(0); 
    }
*/     
    return 0;
}
