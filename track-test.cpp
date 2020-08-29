#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <err.h>
#include <unordered_map>
#include <chrono>
#include <queue>
#include <exception>
#include <algorithm>
#include <random>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaarithm.hpp>
#include <omp.h>

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
        //h_min = { -M_PI/8, -0.125 * w, -0.125 * h,
        //           0.9,  0.9, -0.05, -0.05, -0.0001, -0.0001 };
        //h_max = {  M_PI/8,  0.125 * w,  0.125 * h,
        //           1.1,  1.1,  0.05,  0.05,  0.0001,  0.0001 };
        h_min = { -M_PI/2, -0.25 * w, -0.25 * h,
                   0.8,  0.8, -0.1, -0.1, -0.0003, -0.0003 };
        h_max = {  M_PI/2,  0.25 * w,  0.25 * h,
                   1.2,  1.2,  0.1,  0.1,  0.0003,  0.0003 };
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

template <typename T1, typename T2>
std::ostream& operator<<(std::ostream& os, const std::pair<T1,T2>& p)
{
    os << "{" << p.first << " : " << p.second << "}";
    return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const vector<T>& v)
{
    os << "[";
    for (unsigned i = 0; i < v.size(); ++i)
        os << v[i] << ", ";
    os << "]\n";
    return os;
}

template <typename T, typename Td>
std::pair<T,T> &operator/=(std::pair<T,T> &lhs, const Td &rhs)
{
    lhs.first /= rhs;
    lhs.second /= rhs;
    return lhs;
}

template <typename T>
std::pair<T,T> &operator+=(std::pair<T,T> &lhs, const std::pair<T,T> &rhs)
{
    lhs.first += rhs.first;
    lhs.second += rhs.second;
    return lhs;
}

template <typename T> std::pair<T,T> operator+(std::pair<T,T> lhs, const std::pair<T,T> &rhs)
{
    return lhs += rhs;
}

template <typename T>
vector<T> &operator+=(vector<T> &lhs, const vector<T> &rhs)
{
    if (lhs.size() != rhs.size())
        throw std::length_error("vectors must be same size to add");
    for (size_t i = 0; i < lhs.size(); ++i)
        lhs[i] += rhs[i];
    return lhs;
}

template <typename T> vector<T> operator+(vector<T> lhs, const vector<T> &rhs)
{
    return lhs += rhs;
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
    enum Type {KP_FAST, KP_BRISK, KP_ORB, KP_MSER, KP_AGAST, KP_AKAZE,
               DESC_BRISK, DESC_FREAK, DESC_ORB, DESC_LATCH, DESC_AKAZE,
               UNSET};
    Type type;
    std::string name;
    std::string store_dir = ".";
    vector<double> time_us;
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
    vector<double> getParamsFromIdx(unsigned index);
    unsigned getNumParams();
    void generate(vector<Mat> imgs, vector<vector<KeyPoint>> kp_in, bool verbose);
    bool generateIter(vector<Mat> imgs, vector<vector<KeyPoint>> kp_in, bool verbose);
    virtual void calc(vector<Mat> imgs, vector<vector<KeyPoint>> kp_in) = 0;
    virtual void serialize(std::ostream &os)
    {
        size_t time_size = time_us.size();
        os.write((char *)&time_size, sizeof(time_size));
        os.write((char *)&time_us[0], time_size * sizeof(time_us[0]));
    }
    virtual void deserialize(std::istream &is)
    {
        time_us.clear();
        size_t time_size;
        is.read((char *)&time_size, sizeof(time_size));
        time_us.resize(time_size);
        is.read((char *)&time_us[0], time_size * sizeof(time_us[0]));
    }
    bool load()
    {
        std::string filename = store_dir + "/" + name + ".dat";
        cout << "Trying to load " << filename << endl;
        std::ifstream ifs(filename, std::ifstream::in);
        if (!ifs.good())
            return false;
        cout << "Load successful" << endl;
        deserialize(ifs);
        ifs.close();
        return true;
    }
    void store()
    {
        std::string filename = store_dir + "/" + name + ".dat";
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
            case Type::KP_ORB:
                return "kp_orb";
            case Type::KP_MSER:
                return "kp_mser";
            case Type::KP_AGAST:
                return "kp_agast";
            case Type::KP_AKAZE:
                return "kp_akaze";
            case Type::DESC_BRISK:
                return "desc_brisk";
            case Type::DESC_FREAK:
                return "desc_freak";
            case Type::DESC_ORB:
                return "desc_orb";
            case Type::DESC_LATCH:
                return "desc_latch";
            case Type::DESC_AKAZE:
                return "desc_akaze";
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

void Method::reset()
{
    params = p_min;
    time_us.clear();
}

bool Method::generateIter(vector<Mat> imgs, vector<vector<KeyPoint>> kp_in = {}, bool verbose = true)
{
    if (verbose)
        printParams();
    auto t0 = std::chrono::high_resolution_clock::now();
    calc(imgs, kp_in);
    auto t1 = std::chrono::high_resolution_clock::now();
    time_us.push_back(std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count());
    incrementParams();
    if (params.back() > p_max.back())
        return false;
    setParams(params);
    return true;
}

void Method::generate(vector<Mat> imgs, vector<vector<KeyPoint>> kp_in = {}, bool verbose = true)
{
    reset();
    if (load())
        return;
    setParams(p_min);
    while (generateIter(imgs, kp_in, verbose));
}

vector<double> Method::getParamsFromIdx(unsigned index)
{
    vector<double> p(params.size());
    vector<double> id(params.size());
    vector<double> num_steps(params.size());

    for(unsigned i=0; i<num_steps.size(); i++)
        num_steps[i] = (p_max[i]-p_min[i])/step[i] + 1;

    unsigned tmp = index;
    for(unsigned i=0; i<params.size(); i++) {
        id[i] = tmp % (unsigned)round(num_steps[i]);
        tmp /= num_steps[i];
        p[i] = p_min[i] + step[i]*id[i];
    }
 
    return p; 
}

unsigned Method::getNumParams()
{
    unsigned num_params = 1;
    for(unsigned i=0; i<params.size(); i++) {
        num_params *= round((p_max[i]-p_min[i])/step[i]) + 1;
    }
    return num_params;
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
                    f2d = cv::FastFeatureDetector::create();
                    break;
                case Type::KP_BRISK:
                    f2d = cv::BRISK::create();
                    break;
                case Type::KP_ORB:
                    f2d = cv::ORB::create();
                    break;
                case Type::KP_MSER:
                    f2d = cv::MSER::create();
                    break;
                case Type::KP_AGAST:
                    f2d = cv::AgastFeatureDetector::create();
                    break;
                case Type::KP_AKAZE:
                    f2d = cv::AKAZE::create();
                    break;
                default:
                    err(1,"Unknown KeyPointMethod type");
                    break;
            }
            setParams(p_min);
            name = getTypeName(type);
        }
        void reset()
        {
            Method::reset();
            kp.clear();
            kp_unique.clear();
            hkps.clear();
        }
        void setParams(vector<double> p)
        {
            switch(type) {
                case Type::KP_FAST:
                    {
                    auto f = dynamic_cast<cv::FastFeatureDetector*>(f2d.get());
                    f->setThreshold(p[0]);
                    f->setNonmaxSuppression(p[1]);
                    f->setType(static_cast<cv::FastFeatureDetector::DetectorType>(p[2]));
                    break;
                    }
                case Type::KP_BRISK:
                    f2d = cv::BRISK::create(p[0], p[1]);
                    break;
                case Type::KP_ORB:
                    {
                    auto orb = dynamic_cast<cv::ORB*>(f2d.get());
                    orb->setFastThreshold(p[0]);
                    orb->setMaxFeatures(p[1]);
                    orb->setNLevels(p[2]);
                    orb->setScaleFactor(p[3]);
                    orb->setEdgeThreshold(p[4]);
                    break;
                    }
                case Type::KP_MSER:
                    {
                    auto mser = dynamic_cast<cv::MSER*>(f2d.get());
                    mser->setDelta(p[0]);
                    mser->setMinArea(p[1]);
                    mser->setMaxArea(p[2]);
                    break;
                    }
                case Type::KP_AGAST:
                    {
                    auto a = dynamic_cast<cv::AgastFeatureDetector*>(f2d.get());
                    a->setThreshold(p[0]);
                    a->setNonmaxSuppression(p[1]);
                    a->setType(static_cast<cv::AgastFeatureDetector::DetectorType>(p[2]));
                    break;
                    }
                case Type::KP_AKAZE:
                    {
                    auto a = dynamic_cast<cv::AKAZE*>(f2d.get());
                    a->setThreshold(p[0]);
                    a->setNOctaves(p[1]);                    
                    a->setNOctaveLayers(1 << (int)p[2]);    
                    a->setDiffusivity(static_cast<cv::KAZE::DiffusivityType>(p[2]));                
                    break;
                    }
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
            Method::serialize(os);
            size_t n_imgs = kp.size();
            os.write((char *)&n_imgs, sizeof(n_imgs));
            for (unsigned i = 0; i < n_imgs; i++) {
                size_t kpsize = kp[i].size();
                os.write((char *)&kpsize, sizeof(kpsize));
                for (unsigned j = 0; j < kpsize; j++) {
                    size_t vsize = kp[i][j].size();
                    os.write((char *)&vsize, sizeof(vsize));
                    os.write((char *)&kp[i][j][0], vsize * sizeof(KeyPoint));
                }
            }
        }
        void deserialize(std::istream &is)
        {
            Method::deserialize(is);
            kp.clear();
            size_t n_imgs;
            is.read((char *)&n_imgs, sizeof(n_imgs));
            kp.resize(n_imgs);
            for (unsigned i = 0; i < n_imgs; i++) {
                size_t kpsize;
                is.read((char *)&kpsize, sizeof(kpsize));
                kp[i].resize(kpsize);
                for (unsigned j = 0; j < kpsize; j++) {
                    size_t vsize;
                    is.read((char *)&vsize, sizeof(vsize));
                    kp[i][j].resize(vsize);
                    is.read((char *)&kp[i][j][0], vsize * sizeof(KeyPoint));
                }
            }
            is.read((char *)&time_us[0], time_us.size() * sizeof(time_us[0]));
        }
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
        // TODO: Rename idx to something better!!!
        vector<vector<vector<vector<unsigned>>>> idx = {}; // img - desc - kp - indices in desc mat for same desc & img
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
                case Type::DESC_ORB:
                    f2d = cv::ORB::create();
                    break;
                case Type::DESC_AKAZE:
                    f2d = cv::AKAZE::create();
                    break;
                case Type::DESC_BRISK:
                case Type::DESC_FREAK:
                case Type::DESC_LATCH:
                    break;
                default:
                    err(1,"Unknown DescriptorMethod type at init");
                    break;
            }
            setParams(p_min);
            desc_name = getTypeName(type);
        }
        void reset()
        {
            Method::reset();
            desc.clear();
            idx.clear();
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
                for (KeyPoint const &keypoint : kp_in[i]) {
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
                case Type::DESC_FREAK:
                    f2d = cv::xfeatures2d::FREAK::create(p[0],p[1],p[2],p[3]);
                    break;
                case Type::DESC_ORB:
                        dynamic_cast<cv::ORB*>(f2d.get())->setPatchSize(p[0]);
                    break;
                case Type::DESC_LATCH:
                    f2d = cv::xfeatures2d::LATCH::create(1 << (int)p[2], p[3], p[1], p[0]);
                    break;
                case Type::DESC_AKAZE:
                    break;
                default:
                    err(1,"Unknown DescriptorMethod type at setParams");
                    break;
            }
        }
        void serialize(std::ostream &os)
        {
            Method::serialize(os);
            // Serialize desc
            size_t dsize = desc.size();
            os.write((char *)&dsize, sizeof(dsize));
            for (unsigned i=0; i<dsize; i++) {
                size_t size = desc[i].size();
                os.write((char *)&size, sizeof(size));
                for (unsigned j=0; j<size; j++) {
                    int type = desc[i][i].type();
                    os.write((const char*)(&desc[i][j].rows), sizeof(int));
                    os.write((const char*)(&desc[i][j].cols), sizeof(int));
                    os.write((const char*)(&type), sizeof(int));
                    os.write((const char*)(desc[i][j].data), desc[i][j].elemSize() * desc[i][j].total());
                }
            }

            // Serialize idx
            size_t s0 = idx.size();
            os.write((char *)&s0, sizeof(s0));
            for(auto const &vvv : idx) {
                size_t s1 = vvv.size();
                os.write((char *)&s1, sizeof(s1));
                for(auto const &vv : vvv) {
                    size_t s2 = vv.size();
                    os.write((char *)&s2, sizeof(s2));
                    for(auto const &v : vv) {
                        size_t s3 = v.size();
                        os.write((char *)&s3, sizeof(s3));
                        os.write((char *)&v[0], s3 * sizeof(unsigned));
                    }
                }
            }
        }
        void deserialize(std::istream &is)
        {
            Method::deserialize(is);
            desc.clear();
            size_t dsize;
            is.read((char *)&dsize, sizeof(dsize));
            desc.resize(dsize);
            for (unsigned i=0; i<dsize; i++) {
                size_t size;
                is.read((char *)&size, sizeof(size));
                desc[i].resize(size);
                for (unsigned j=0; j<size; j++) {
                    int rows, cols, type;
                    is.read((char*)(&rows), sizeof(int));
                    is.read((char*)(&cols), sizeof(int));
                    is.read((char*)(&type), sizeof(int));

                    desc[i][j].create(rows, cols, type);
                    is.read((char*)(desc[i][j].data), desc[i][j].elemSize() * desc[i][j].total());
                }
            }

            // Deserialize idx
            size_t s0;
            is.read((char *)&s0, sizeof(s0));
            idx.resize(s0);
            for(unsigned i=0; i<s0; i++) {
                size_t s1;
                is.read((char *)&s1, sizeof(s1));
                idx[i].resize(s1);
                for(unsigned j=0; j<s1; j++) {
                    size_t s2;
                    is.read((char *)&s2, sizeof(s2));
                    idx[i][j].resize(s2);
                    for(unsigned k=0; k<s2; k++) {
                        size_t s3;
                        is.read((char *)&s3, sizeof(s3));
                        idx[i][j][k].resize(s3);
                        is.read((char *)&idx[i][j][k][0], s3 * sizeof(unsigned));
                    }
                }
            }
        }
        void reconstruct_idx (vector<vector<KeyPoint>> &kp_unique, vector<std::unordered_map<HKeyPoint, vector<unsigned>, HKeyPointHasher>> &hkps, unsigned orig_len)
        {
            // reconstruct from desc_params-imgs-descriptor_mat
            // kp is desc-params-imgs-kp
            // reconstruct to imgs-desc_params-kp_params-descriptor_mat
            if (idx.size() > 0)
                return;
            cout << "Reconstructing" << endl;
            idx.resize(desc[0].size(),vector<vector<vector<unsigned>>>(desc.size(),vector<vector<unsigned>>(orig_len)));
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
                        for (unsigned const &id : kp_idx)
                            idx[j][i][id].push_back(k);
                    }
                }
            }
        }
        Mat getDesc(unsigned img_idx, unsigned desc_idx, unsigned kp_idx)
        {
            vector<unsigned> row_ids = idx[img_idx][desc_idx][kp_idx];
            unsigned desc_len = desc[0][0].cols;
            unsigned desc_count = row_ids.size();
            if (desc_count == 0 || desc_len == 0)
                return Mat();
            Mat M = Mat(desc_count, desc_len, desc[0][0].depth());
            for (unsigned i=0; i<row_ids.size(); i++)
                desc[desc_idx][img_idx].row(row_ids[i]).copyTo(M.row(i));
            return M;
        }
        vector<KeyPoint> getKp(vector<KeyPoint> kp_unique, unsigned img_idx, unsigned desc_idx, unsigned kp_idx)
        {
            vector<KeyPoint> K;
            for (unsigned i : idx[img_idx][desc_idx][kp_idx])
                K.push_back(kp_unique[kp_id[desc_idx][img_idx][i]]);
            return K;
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
            if (desc.size() == 0)
                err(1, "desc not calculated, cannot store it.");
            
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
    if (H.empty() || Href.empty())
        return 500; // Large penalty, but not infinite 
    vector<cv::Point2d> pref, pH;
    double err = 0;
    perspectiveTransform(p, pref, Href);
    perspectiveTransform(p, pH, H);
    for (unsigned i = 0; i < p.size(); i++) {
        cv::Point2d diff = pref[i] - pH[i];
        err += sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    err /= p.size();
    // Cap the error so 1 bad classification is punished, but not infinitely
    if (err > 500)
        err = 500; 
    return err;
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

class MyFlann : public cv::FlannBasedMatcher {
public:
void knnMatchImpl( cv::InputArray _queryDescriptors, std::vector<std::vector<cv::DMatch> >& matches, int knn) {cv::FlannBasedMatcher::knnMatchImpl(_queryDescriptors,matches,knn);}
};

vector<vector<vector<cv::DMatch>>> matchDescs(unsigned n_ref, unsigned n_gen, vector<Mat> d) {

    vector<unsigned> ref_id(n_ref*n_gen), gen_id(n_ref*n_gen);
    for (unsigned i = 0; i < n_ref*n_gen; i++) {
        ref_id[i] = i / n_gen * (n_gen + 1);
        gen_id[i] = i / n_gen * (n_gen + 1) + i % n_gen + 1;
    }

    bool gpu = false;
    vector<vector<vector<cv::DMatch>>> matches(n_ref*n_gen);

    if (gpu && d[0].depth() != CV_32F) {
        cv::cuda::Stream stream;
        cv::Ptr<cv::cuda::DescriptorMatcher> m_gpu = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
        vector<cv::cuda::GpuMat> d_gpu(d.size());
        vector<cv::cuda::GpuMat> gpu_matches(n_ref*n_gen);
        for (unsigned i=0; i<d.size(); i++)
            d_gpu[i].upload(d[i]);
        for (unsigned i = 0; i < n_ref*n_gen; i++) {
            if (d_gpu[ref_id[i]].rows > 3 && d_gpu[gen_id[i]].rows > 3)
                m_gpu->knnMatchAsync(d_gpu[ref_id[i]], d_gpu[gen_id[i]],
                                     gpu_matches[i], 2, cv::noArray(), stream);
        }
        stream.waitForCompletion();
        for (unsigned i = 0; i < n_ref*n_gen; i++)
            m_gpu->knnMatchConvert(gpu_matches[i], matches[i]);
    } else {
        #pragma omp parallel for
        for (unsigned i = 0; i < d.size(); i++)
            d[i].convertTo(d[i],CV_32F);
        vector<MyFlann> matchers(n_ref*n_gen);

        #pragma omp parallel for
        for (unsigned i = 0; i < n_ref*n_gen; i++) {
            if(d[ref_id[i]].rows > 2)
                matchers[i].add(d[ref_id[i]]);
        }
        // For some reason training once for each ref produces different
        // results than training for each pair
        // Maybe it's harmless, but I don't want to risk flawed results
        for (unsigned i = 0; i < n_ref*n_gen; i++)
            if(d[ref_id[i]].rows > 2 && d[gen_id[i]].rows > 2)
                matchers[i].train();
        #pragma omp parallel for
        for (unsigned i = 0; i < n_ref*n_gen; i++)
            if(d[ref_id[i]].rows > 2 && d[gen_id[i]].rows > 2)
                matchers[i].knnMatchImpl(d[gen_id[i]],matches[i],2);
    }
    return matches;
}

std::pair<vector<cv::Point2f>,vector<cv::Point2f>> selectGoodMatches(unsigned n_ref, unsigned n_gen, vector<KeyPoint> fromkp, vector<KeyPoint> tokp, vector<vector<cv::DMatch>> matches, double thresh)
{
    vector<cv::DMatch> good_matches;
    for (auto match : matches)
        if (match[0].distance < thresh * match[1].distance)
            good_matches.push_back(match[0]);

    vector<cv::Point2f> toP(good_matches.size()), fromP(good_matches.size());
    for (unsigned i=0; i<good_matches.size(); i++) {
        fromP[i] = fromkp[good_matches[i].trainIdx].pt;
        toP[i] = tokp[good_matches[i].queryIdx].pt;
    }
    return {fromP, toP};
}

vector<Mat> findHomographies(unsigned n_ref, unsigned n_gen, vector<Mat> d, vector<vector<KeyPoint>> kp)
{
    vector<Mat> H(n_ref*n_gen);
    vector<vector<vector<cv::DMatch>>> matches = matchDescs(n_ref,n_gen,d);

    #pragma omp parallel for
    for (unsigned i=0; i<H.size(); i++) {
        auto gm = selectGoodMatches(n_ref, n_gen, kp[i / n_gen * (n_gen + 1)],
                  kp[i / n_gen * (n_gen + 1) + i % n_gen + 1],matches[i], 0.88);
        if (gm.first.size() < 4)
            continue;

        H[i] = cv::findHomography(gm.first, gm.second, cv::RANSAC, 6, cv::noArray(), 6000, 0.99);
    }
    return H;
}

void testDescriptorKeypointMap(vector<vector<vector<Mat>>> &uniq, vector<vector<vector<Mat>>> &brute, vector<vector<vector<vector<KeyPoint>>>> uniq_kp, vector<vector<vector<vector<KeyPoint>>>> brute_kp)
{
    cout << "Unique: " << uniq.size() << ", brute: " << brute.size() << endl; 
    cout << "Unique kp: " << uniq_kp.size() << ", brute kp: " << brute_kp.size() << endl; 
    if (uniq.size() != brute.size() || uniq_kp.size() != brute_kp.size())
        err(1, "Unique and brute size mismatch");
    for (unsigned i = 0; i < uniq.size(); i++) { // imgs
        cout << "Unique[" << i << "] : " << uniq[i].size() << ", brute[" << i << "]: " << brute[i].size() << endl; 
        cout << "Unique kp[" << i << "] : " << uniq_kp[i].size() << ", brute kp[" << i << "]: " << brute_kp[i].size() << endl; 
        if (uniq[i].size() != brute[i].size() || uniq_kp[i].size() != brute_kp[i].size())
            err(1, "Unique and brute size mismatch");
        for (unsigned j = 0; j < uniq[i].size(); j++) { // descs
            cout << "Unique[" << i << "][" << j << "] : " << uniq[i][j].size() << ", brute[" << i << "][" << j << "] : " << brute[i][j].size() << endl; 
            cout << "Unique kp[" << i << "][" << j << "] : " << uniq_kp[i][j].size() << ", brute kp[" << i << "][" << j << "] : " << brute_kp[i][j].size() << endl; 
            if (uniq[i][j].size() != brute[i][j].size() || uniq[i][j].size() != brute[i][j].size())
                err(1, "Unique and brute size mismatch");
            for (unsigned k = 0; k < uniq[i][j].size(); k++) { // kps
                cout << "Unique[" << i << "][" << j << "][" << k << "] : " << uniq[i][j][k].rows << ", brute[" << i << "][" << j << "][" << k << "] : " << brute[i][j][k].rows << endl; 
                cout << "Unique kp[" << i << "][" << j << "][" << k << "] : " << uniq_kp[i][j][k].size() << ", brute kp[" << i << "][" << j << "][" << k << "] : " << brute_kp[i][j][k].size() << endl; 
                if (uniq[i][j][k].rows != brute[i][j][k].rows || uniq_kp[i][j][k].size() != brute_kp[i][j][k].size())
                    err(1, "Unique and brute size mismatch");
                Mat res;
                cv::sort(uniq[i][j][k],uniq[i][j][k], cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);
                cv::sort(brute[i][j][k],brute[i][j][k], cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);
                cv::compare(uniq[i][j][k],brute[i][j][k], res, cv::CMP_NE);
                if (cv::sum(res)[0])
                    cout << "Diff is: " << cv::sum(res)[0] << endl;
                for (unsigned l = 0; l < uniq_kp[i][j][k].size(); l++) {
                    cout << uniq[i][j][k].row(l) << endl;
                    cout << uniq_kp[i][j][k][l] << endl;
                    cout << brute[i][j][k].row(l) << endl;
                    cout << brute_kp[i][j][k][l] << endl;
                    cout << endl;
                }    
            }
        }
    }
}

void test(vector<vector<Mat>> I, KeyPointMethod *kpm,
          DescriptorMethod *dm)
{
    vector<vector<vector<Mat>>> uniq, brute; // I, kp, desc
    vector<vector<vector<vector<KeyPoint>>>> uniq_kp, brute_kp;

    for (unsigned i = 0; i < I.size(); i++) {
        kpm->generate(I[i], {});
        kpm->getUnique();
        dm->setKpMethodType(kpm->type);
        vector<vector<KeyPoint>> tmp = kpm->kp_unique;
        cout << "Generating from unique kp" << endl;
        dm->generate(I[i], tmp);
        dm->reconstruct_idx(tmp, kpm->hkps, kpm->kp.size());
        uniq.resize(I[i].size(),vector<vector<Mat>>(dm->desc.size(),vector<Mat>(kpm->kp.size())));
        uniq_kp.resize(I[i].size(),vector<vector<vector<KeyPoint>>>(dm->desc.size(),vector<vector<KeyPoint>>(kpm->kp.size())));
        for (unsigned i=0; i<uniq.size(); i++)
            for (unsigned j=0; j<uniq[i].size(); j++)
                for (unsigned k=0; k<uniq[i][j].size(); k++) {
                    uniq[i][j][k] = dm->getDesc(i,j,k);
                    uniq_kp[i][j][k] = dm->getKp(kpm->kp_unique[i], i,j,k);
                    }
        cout << "Generating from brute kp" << endl;
        dm->generate(I[i], kpm->kp[0]);
        brute.resize(I[i].size(),vector<vector<Mat>>(dm->desc.size(),vector<Mat>(kpm->kp.size())));
        brute_kp.resize(I[i].size(),vector<vector<vector<KeyPoint>>>(dm->desc.size(),vector<vector<KeyPoint>>(kpm->kp.size())));
        for (unsigned j = 0; j < kpm->kp.size(); j++) {
            dm->generate(I[i], kpm->kp[j]);
            for (unsigned k = 0; k < dm->desc.size(); k++) {
                for (unsigned l = 0; l < dm->desc[k].size(); l++) {
                    brute[l][k][j] = dm->desc[k][l];
                    for(unsigned id : dm->kp_id[k][l])
                        brute_kp[l][k][j].push_back(kpm->kp[j][l][id]);
                }
            }
        }
        testDescriptorKeypointMap(uniq, brute, uniq_kp, brute_kp);
        brute.clear();  
        uniq.clear();
        brute_kp.clear();
        uniq_kp.clear();
    }
}

vector<vector<std::pair<double,double>>> evaluateKpDesc(vector<vector<Mat>> imgs, vector<Mat> Hgen, KeyPointMethod *kpm, DescriptorMethod *dm)
{
    // Assume all rows of imgs are the same size
    unsigned n_ref_imgs = imgs.size();
    unsigned n_gen_imgs = imgs[0].size()-1;
    unsigned n_kp = kpm->getNumParams();
    unsigned n_desc = dm->getNumParams();

    vector<vector<std::pair<double,double>>> score(n_desc, vector<std::pair<double,double>> (n_kp, std::make_pair<double,double>(0,0)));

    vector<Mat> I(n_ref_imgs*(n_gen_imgs+1));
    vector<double> kp_time(n_kp);
    vector<double> desc_time(n_desc);

    // Flatten 2D img vector to 1D
    for (unsigned i = 0; i < n_ref_imgs; i++)
        std::copy(imgs[i].begin(), imgs[i].end(), I.begin()+i*imgs[i].size());

    kpm->generate(I, {});
    kpm->store();
    kpm->getUnique();
    dm->setKpMethodType(kpm->type);
    vector<vector<KeyPoint>> kp_unique = kpm->kp_unique;
    kp_time = kpm->time_us;

    // Explode generate function for descriptors,
    // not enough memory to store it with a single run
    // So calculate the score right after generation, then delete desc
    for (unsigned i = 0; i < n_desc; i++) {
        // Partial reset - keep kp_map, keypoints are constant
        dm->time_us.clear(); 
        dm->desc.clear();
        dm->idx.clear();
        dm->kp_id.clear();

        dm->generateIter(I, kpm->kp_unique);
        dm->reconstruct_idx(kpm->kp_unique, kpm->hkps, n_kp);
        desc_time[i] = dm->time_us[0];
        unsigned descs_total = 0;
        for (Mat dmat : dm->desc[0])
            descs_total += dmat.rows;
        for (unsigned j = 0; j < n_kp; j++) {
            cout << "Calculating score of desc " << i + 1 << "/" << n_desc << ", kp " << j + 1 << "/" << n_kp << endl;
            vector<Mat> descs(I.size());
            vector<vector<KeyPoint>> kps(I.size());
            #pragma omp parallel for
            for (unsigned k = 0; k < descs.size(); k++) {
                kps[k] = dm->getKp(kpm->kp_unique[k], k, 0, j);
                descs[k] = dm->getDesc(k, 0, j);
            }
            vector<Mat> H = findHomographies(n_ref_imgs, n_gen_imgs, descs, kps);
            for (unsigned k=0; k<H.size(); k++) {
                    score[i][j].first += meanTransformError(Hgen[k], H[k], test_points); 
                    //showTransformError(I[k / n_gen_imgs * (n_gen_imgs + 1) + k % n_gen_imgs + 1], Hgen[k], H[k], test_points);
                    unsigned ref_id = k / n_gen_imgs * (n_gen_imgs + 1);
                    unsigned gen_id = ref_id + k % n_gen_imgs + 1;
                    score[i][j].second += desc_time[i] / descs_total
                        * (descs[ref_id].rows + descs[gen_id].rows);
            }
            score[i][j].first /= (n_ref_imgs * n_gen_imgs);
            score[i][j].second += kp_time[j];
            score[i][j].second /= (I.size());
        }
    }
    return score; 
}

void storeScore(vector<vector<std::pair<double,double>>> score, std::string basename)
{
    std::string filename = basename + "_score.csv";
    std::ofstream f0(filename, std::ofstream::out);
    filename = basename + "_time.csv";
    std::ofstream f1(filename, std::ofstream::out);

    for(auto const &v : score) {
        f0 << v[0].first;
        f1 << v[0].second;
        for(unsigned i=1; i<v.size(); i++) {
            f0 << ", " << v[i].first;
            f1 << ", " << v[i].second;
        }
        f0 << "\n";
        f1 << "\n";
    }
    f0.close();
    f1.close();
}

// TODO: Come up with a better name!
void scoreMethodVariants(vector<vector<Mat>> I, vector<Mat> H, vector<KeyPointMethod> kpms, vector<DescriptorMethod> dms, std::string result_dir)
{
    for (auto &kpm : kpms) {
        for(auto &dm : dms) {
            kpm.store_dir = result_dir + "/dat";
            dm.store_dir = result_dir + "/dat";
            dm.setKpMethodType(kpm.type);
            std::string csv_name = result_dir + "/csv/" + dm.name;
            std::string saved_filename = csv_name  + "_score.csv";
            if  (access(saved_filename.c_str(), F_OK ) != -1) {
                cout << "Skipping score calc for " << dm.name << " - already calculated" << endl;
                continue;
            }
            vector<vector<std::pair<double,double>>> score = evaluateKpDesc(I, H, &kpm, &dm);
            storeScore(score, csv_name);
            dm.reset();
        }
    }    
}

// Load csv without header, consisting of only numbers 
std::vector<std::vector<double>> loadCSV(std::string filename){

    std::vector<std::vector<double>> vals;
    std::ifstream f(filename);
    if (!f.is_open())
        throw std::runtime_error("Could not open file");

    std::string line;
    int row_idx = 0;
    while (std::getline(f, line)) {
        vals.push_back({});
        std::stringstream ss(line);
        int col_idx = 0;
        double val;
        while (ss >> val) {
            vals[row_idx].push_back(val);
            // If the next token is a comma, ignore it and move on
            if (ss.peek() == ',')
                ss.ignore();
            col_idx++;
        }
        row_idx++;
    }
    f.close();

    return vals;
}

cv::Mat preprocess(cv::Mat input)
{
    cv::Mat im = input.clone();
    // Median filter + contrast limited histogram equalization
    cv::medianBlur(im, im, 3); 
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(18, cv::Size(8,8));
    clahe->apply(im, im);
    // Image sharpening by unsharp masking with gaussian blur
    cv::Mat sh; 
    cv::GaussianBlur(im, sh, cv::Size(0, 0), 7, 7); 
    cv::addWeighted(im, 1.5, sh, -0.5, 0, sh);
    return im; 
}

vector<KeyPointMethod> getKpMethods(bool quick = false) {
    vector<KeyPointMethod> v;
    vector<vector<double>> fast_p, brisk_p, orb_p, mser_p, agast_p, akaze_p;
    if(quick) {
        fast_p = {{40,1,2},{42,1,2},{1,1,1}};
        brisk_p = {{40,0},{41,1},{1,1}};
        orb_p = {{40,500,1,1.2,31},{41,550,1,1.2,31},{1,50,1,0.2,2}};
        mser_p = {{5,64,12000},{6,64,14000},{1,16,2000}};
        agast_p = {{20,1,3},{20,1,3},{1,1,1}};
        akaze_p = {{0.001,1,1,0},{0.001,1,1,0},{0.0002,1,1,1}};
    } else {
        fast_p = {{10,0,2},{30,1,2},{1,1,1}};
        brisk_p = {{5,3},{35,4},{1,1}};
        vector<double> orb_default = {20,500,8,1.2,31};
        orb_p = {{10,650,4,1.4,33},{40,800,8,1.4,33},{1,50,1,0.2,2}};
        mser_p = {{1,16,3500},{3,32,4500},{1,16,500}};
        agast_p = {{10,0,3},{30,0,3},{1,1,1}};
        akaze_p = {{0.001,1,2,0},{0.003,3,2,3},{0.001,1,1,1}};
    }
    KeyPointMethod fast(Method::Type::KP_FAST, 3,fast_p[0],fast_p[1],fast_p[2]);
    KeyPointMethod brisk(Method::Type::KP_BRISK,2,brisk_p[0],brisk_p[1],brisk_p[2]);
    KeyPointMethod orb(Method::Type::KP_ORB,5,orb_p[0],orb_p[1],orb_p[2]);
    KeyPointMethod mser(Method::Type::KP_MSER,3,mser_p[0],mser_p[1],mser_p[2]);
    KeyPointMethod agast(Method::Type::KP_AGAST,3,agast_p[0],agast_p[1],agast_p[2]);
    KeyPointMethod akaze(Method::Type::KP_AKAZE,4,akaze_p[0],akaze_p[1],akaze_p[2]);
    return {fast,brisk,mser,agast,akaze,orb};
}

vector<DescriptorMethod> getDescMethods(bool quick = false) {
    vector<DescriptorMethod> v;
    vector<vector<double>> brisk_p, freak_p, orb_p, latch_p, akaze_p;
    if(quick) {
        brisk_p = {{1},{1},{0.1}};
        freak_p = {{0,0,21,3},{0,0,22,4},{1,1,1,1}};
        orb_p = {{29},{31},{2}};
        latch_p= {{1.4,3,5,0},{1.4,3,5,0},{0.1,1,1}}; 
        akaze_p = {{1},{1},{1}};
    } else {
        brisk_p = {{0.7},{1.3},{0.1}};
        freak_p = {{0,1,22,3},{0,1,32,5},{1,1,2,1}};
        orb_p = {{29},{31},{2}};
        latch_p= {{1.4,3,6,1},{1.6,4,6,1},{0.2,1,1,1}};
        akaze_p = {{1},{1},{1}};
    }
    DescriptorMethod brisk(Method::Type::DESC_BRISK,1,brisk_p[0],brisk_p[1],brisk_p[2]);
    DescriptorMethod freak(Method::Type::DESC_FREAK,4,freak_p[0],freak_p[1],freak_p[2]);
    DescriptorMethod orb(Method::Type::DESC_ORB,1,orb_p[0],orb_p[1],orb_p[2]);
    DescriptorMethod latch(Method::Type::DESC_LATCH,4,latch_p[0],latch_p[1],latch_p[2]);
    DescriptorMethod akaze(Method::Type::DESC_AKAZE,1,akaze_p[0],akaze_p[1],akaze_p[2]);
    return {brisk, freak, orb, latch};
}

int main()
{
    //srand (time(NULL));
    srand (0);

    vector<cv::String> fn;
    cv::glob("../images/temperature_invariant/*.png", fn, false);
    vector<Mat> imgs;
    size_t count = fn.size(); //number of png files in images folder
    for (size_t i=0; i<count; i++)
        imgs.push_back(cv::imread(fn[i],cv::IMREAD_GRAYSCALE));

    auto rng = std::default_random_engine {};
    std::shuffle(std::begin(imgs), std::end(imgs), rng);

    for (size_t i=0; i<328; i++)
        imgs.pop_back();
    unsigned n_refs = 1;
    unsigned n_gen = (imgs.size() - n_refs) / n_refs;
    init(imgs[0].cols, imgs[0].rows);
    vector<Mat> H = generateHomographies(imgs.size()-n_refs);
    //vector<vector<Mat>> I = generateTransformedImgs(H, imgs);
    vector<vector<Mat>> I(n_refs,vector<Mat>(n_gen + 1));
    for (unsigned i = 0; i < n_refs; i++) {
        I[i][0] = imgs[i * (n_gen + 1)];
        for (unsigned j = 1; j < n_gen + 1; j++)
            cv::warpPerspective(imgs[i * (n_gen + 1) + j - 1], I[i][j], H[i * n_gen + j - 1], imgs[i * (n_gen + 1) + j - 1].size());
    }

    for (auto& img_row : I)
        for (auto& img : img_row)
            img = preprocess(img);

    cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(18,false,cv::FastFeatureDetector::TYPE_9_16);
    cv::Ptr<cv::BRISK> brisk = cv::BRISK::create(0,0,1.8);
    
    vector<Mat> I_1D(imgs.size());
    for (unsigned i = 0; i < n_refs; i++)
        std::copy(I[i].begin(), I[i].end(), I_1D.begin()+i*I[i].size());
    vector<Mat> I_1D_orig = I_1D;

    vector<std::pair<double,double>> score;
    for (unsigned i=0; i<I_1D.size(); i++)
        I_1D[i] = preprocess(I_1D_orig[i]);
/*  for (unsigned i=0; i<I_1D.size(); i++) {
    cv::imshow("LA",I_1D[i]);
    cv::waitKey(0);
    }
    exit(0);
*/

    vector<vector<KeyPoint>> kp;
    vector<Mat> desc;
    fast->detect(I_1D,kp);
    brisk->compute(I_1D,kp,desc);
    auto t0 = std::chrono::high_resolution_clock::now();
    vector<Mat> Hcalc = findHomographies(n_refs, n_gen, desc, kp);
    auto t1 = std::chrono::high_resolution_clock::now();
    double time = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count();
    double tmpscore = 0;
    for(unsigned i=0; i<H.size(); i++)
        tmpscore += meanTransformError(Hcalc[i], H[i], test_points); 
    tmpscore /= I_1D.size();
    cout << "Score: " << tmpscore << ", time: " << time << " us\n"; exit(0);

    return 0;
}
