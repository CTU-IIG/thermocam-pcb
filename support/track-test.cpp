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
h_params h_min_easy, h_max_easy, h_min_hard, h_max_hard;
bool initialized = false;

void init(int img_width, int img_height)
{
    double w = (double)img_width;
    double h = (double)img_height;

    // Init centering matrix and inverse
    C =  (cv::Mat_<double>(3, 3) << 1, 0,  w / 2, 0, 1,  h / 2, 0, 0, 1);
    iC = (cv::Mat_<double>(3, 3) << 1, 0, -w / 2, 0, 1, -h / 2, 0, 0, 1);

    // Init point grid for testing homography accuracy
    int grid_p = 20;
    double step_x = w/grid_p;
    double step_y = h/grid_p;
    for (int i = 0; i < grid_p; i++)
        for (int j = 0; j < grid_p; j++)
            test_points.push_back({ i * step_x, j * step_y });

    // Init minimum and maximum homography parameters
    h_min_easy = { -M_PI/16, -0.125 * w, -0.125 * h,
                    0.9,  0.9, -0.05, -0.05, -0.0001, -0.0001 };
    h_max_easy = {  M_PI/16,  0.125 * w,  0.125 * h,
                    1.1,  1.1,  0.05,  0.05,  0.0001,  0.0001 };
    h_min_hard = { -M_PI/6, -0.20 * w, -0.20 * h,
                    0.9,  0.9, -0.05, -0.05, -0.0001, -0.0001 };
    h_max_hard = {  M_PI/6,  0.20 * w,  0.20 * h,
                    1.1,  1.1,  0.05,  0.05,  0.0001,  0.0001 };
}

void show(Mat img, cv::String window_name = "Image")
{
    cv::imshow(window_name,img);
    cv::waitKey(0);
}

void show(vector<Mat> imgs, cv::String name = "")
{

    for (unsigned i=0; i<imgs.size(); i++) {
        show(imgs[i], name);
    }
}

void show(vector<vector<Mat>> imgs, cv::String name = "")
{
    for (unsigned i=0; i<imgs.size(); i++) {
        show(imgs[i], name);
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

template <typename T> T vsum(vector<T> v)
{
    if (v.size() < 1)
        throw std::length_error("cannot sum vector of size smaller than 1");
        
    T sum = v[0];
    for (unsigned i=1; i<v.size(); i++)
        sum += v[i];
    return sum;
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

Mat randomHomography(bool hard = true)
{
    h_params *min = (hard) ? &h_min_hard : &h_min_easy;  
    h_params *max = (hard) ? &h_max_hard : &h_max_easy;  
    h_params h = randomHomographyParams(min, max);
    return getHomography(&h);
}

void showTransformError(Mat img, Mat H_ref, Mat H,
                        vector<cv::Point2d> p)
{
    Mat img_ref;
    vector<cv::Point2d> pref, pH;
    cv::warpPerspective(img, img_ref, H_ref, img.size());
    if (img_ref.channels() == 1)
        cv::cvtColor(img_ref,img_ref, cv::COLOR_GRAY2BGR);
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

vector<double> meanTransformError(vector<Mat> H, vector<Mat> Href, vector<cv::Point2d> p)
{
    vector<double> ret(H.size());
    for (unsigned i=0; i < H.size(); i++)
        ret[i] = meanTransformError(H[i], Href[i], p);
    return ret;
}

vector<Mat> generateHomographies(int count, int w, int h, bool hard = true)
{
    init(w,h);
    vector<Mat> H(count);
    for (int i = 0; i < count; i++)
        H[i] = randomHomography(hard);
    return H;
}

Mat warpWithNoise(Mat img, Mat H)
{
    cv::Scalar mean,stddev;
    meanStdDev(img, mean, stddev);
    Mat I = img.clone();
    randu(I, 0, 255);
    cv::warpPerspective(img, I, H, I.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
    return I;
}

vector<Mat> warpWithNoise(vector<Mat> I, vector<Mat> H)
{
    vector<Mat> I_w(I.size());

    #pragma omp parallel for    
    for (unsigned i = 0; i < I.size(); i++)
        I_w[i] = warpWithNoise(I[i],H[i]);
    return I_w;
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

    vector<vector<vector<cv::DMatch>>> matches(n_ref*n_gen);
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
    return matches;
}

std::pair<vector<cv::Point2f>,vector<cv::Point2f>> selectGoodMatches(vector<KeyPoint> fromkp, vector<KeyPoint> tokp, vector<vector<cv::DMatch>> matches, double thresh)
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

vector<Mat> findHomographies(unsigned n_ref, unsigned n_gen, vector<Mat> d, vector<vector<KeyPoint>> kp, cv::UsacParams usacparams = cv::UsacParams())
{
    vector<Mat> H(n_ref*n_gen);
    vector<double> times(n_ref*n_gen);
    vector<vector<vector<cv::DMatch>>> matches = matchDescs(n_ref,n_gen,d);

    //#pragma omp parallel for
    for (unsigned i=0; i<H.size(); i++) {
        auto gm = selectGoodMatches(kp[i / n_gen * (n_gen + 1)],
                  kp[i / n_gen * (n_gen + 1) + i % n_gen + 1],matches[i], 0.88);
        if (gm.first.size() < 4)
            continue;

        auto t0 = std::chrono::high_resolution_clock::now();
        H[i] = cv::findHomography(gm.first, gm.second, cv::noArray(), usacparams);
        //H[i] = cv::findHomography(gm.first, gm.second, cv::RANSAC, 6, cv::noArray(), 6000, 0.99);
        auto t1 = std::chrono::high_resolution_clock::now();
        times[i] = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count();
    }
    cout << vsum(times) / times.size();
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

void writeCSV(std::string filename, vector<vector<double>> vals){

    std::ofstream f(filename);
    if (!f.is_open())
        throw std::runtime_error("Could not open file");

    for(auto const &v : vals) {
        f << v[0];
        for(unsigned i=1; i<v.size(); i++) {
            f << ", " << v[i];
        }
        f << "\n";
    }
    f.close();
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
    if (im.channels() == 3)
        cv::cvtColor(im,im,cv::COLOR_RGB2GRAY);
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

vector<Mat> preprocess(vector<Mat> imgs)
{
    vector<Mat> ret(imgs.size());
    for (unsigned i = 0; i < imgs.size(); i++)
        ret[i] = preprocess(imgs[i]);
    return ret;
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

vector<Mat> read_imgs(cv::String path, bool shuffle = true)
{
    vector<cv::String> fn;
    vector<Mat> imgs;

    cv::glob(path, fn, false);
    size_t count = fn.size(); //number of png files in images folder
    for (size_t i=0; i<count; i++)
        imgs.push_back(cv::imread(fn[i],cv::IMREAD_GRAYSCALE));

    if (shuffle) {
        auto rng = std::default_random_engine {};
        std::shuffle(std::begin(imgs), std::end(imgs), rng);
    }

    return imgs;
}

Mat add_noise(Mat I, double sigma, double noise_blur)
{
    Mat img = I.clone();
    int orig_depth = I.depth();
    Mat noise = Mat(I.size(),CV_16S);
    cv::randn(noise, 0, sigma);
    cv::blur(noise,noise,cv::Size(noise_blur,noise_blur));
    //cv::GaussianBlur(noise,noise,cv::Size(noise_blur,noise_blur),0);
    img.convertTo(img,CV_16S);
    img += noise;
    img.convertTo(img,orig_depth);
    return img;    
}

vector<Mat> add_noise(vector<Mat> I, double sigma, double noise_blur)
{
    #pragma omp parallel for
    for (Mat &img : I)
        img = add_noise(img, sigma, noise_blur);
    return I;
}

vector<Mat> read_vid(cv::String vid_path)
{
    if (access(vid_path.c_str(), F_OK) == -1) // File does not exist
        err(1,"Video open");
    auto video = new cv::VideoCapture(vid_path);

    vector<Mat> I;
    while(1) {
        Mat img;
        *video >> img;
        if (img.empty())
            break;
        I.push_back(img); 
    }

    return I;
}

void onMouse(int event, int x, int y, int flags, void *param)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        vector<cv::Point2f> &p = *((vector<cv::Point2f> *)(param));
        p.push_back({ (float)x, (float)y });
    }       
}

Mat manualSelectHomography(Mat I_ref, Mat I)
{
    cv::String window_name = "Enter correspondences: ref | vid | back | new";
    cv::namedWindow(window_name , cv::WINDOW_NORMAL);
    vector<cv::Point2f> p_ref, p_vid, p_temp;
    setMouseCallback(window_name, onMouse, &p_temp);
    Mat H_manual = Mat::eye(3,3,CV_64F);

    cv::RNG rng(12345);
    vector<cv::Scalar> rc; // random colors
    for (int i=0; i<100; i++)
        rc.push_back(cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255)));

    bool last_ref;
    while(1) {
        if (p_temp.size() > 0) {
            if (p_temp[0].x < I.cols) { // Click in reference
                p_ref.push_back(p_temp[0]);
                last_ref = true;
            }
            else if (p_temp[0].x < 2 * I.cols) {
                p_temp[0].x -= I.cols;
                p_vid.push_back(p_temp[0]);
                last_ref = false;
            }
            p_temp.pop_back();
        }

        Mat ref_draw = I_ref.clone();
        Mat I_draw = I.clone();

        if (ref_draw.channels() == 1)
            cvtColor(ref_draw,ref_draw,cv::COLOR_GRAY2RGB);
        if (I_draw.channels() == 1)
            cvtColor(I_draw,I_draw,cv::COLOR_GRAY2RGB);

        for (unsigned j=0; j<p_ref.size(); j++)
            circle(ref_draw, p_ref[j] , 2, rc[j], -1); 
        for (unsigned j=0; j<p_vid.size(); j++)
            circle(I_draw, p_vid[j] , 2, rc[j], -1); 

        if (p_ref.size() >= 4 && p_vid.size() >= 4 && p_ref.size() == p_vid.size())
            H_manual = cv::findHomography(p_ref, p_vid, cv::RANSAC, 6, cv::noArray(), 6000, 0.99);
        Mat I_tr;
        cv::warpPerspective(I, I_tr, H_manual, I_ref.size(),cv::WARP_INVERSE_MAP);

        if (I_tr.channels() == 1)
            cvtColor(I_tr,I_tr,cv::COLOR_GRAY2RGB);
        Mat concat;
        vector<Mat> v = {ref_draw,I_draw,I_tr};
        cv::hconcat(v,concat);
        cv::imshow(window_name,concat);
        char key = cv::waitKey(1) & 0xEFFFFF;
        if (key == 8) { // Backspace - erase points
            if (last_ref && p_ref.size() > 0)
                p_ref.pop_back();
            else if (p_vid.size() > 0)
                p_vid.pop_back(); 
        }
        if (key == 27) {// Esc - cancel, exit without save
            H_manual = Mat();
            break;
        }
        if (key == 32) // Space - selection ok, exit and save
            break;
    }
    cv::destroyWindow(window_name);
    return H_manual;
}

// Find transformations in video, reference is 1st image
vector<Mat> createTestTransformations(vector<Mat> I, vector<Mat> H_load = {})
{
    cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(18,false,cv::FastFeatureDetector::TYPE_9_16);
    cv::Ptr<cv::BRISK> brisk = cv::BRISK::create(0,0,1.8);

    vector<vector<KeyPoint>> kp(2);
    vector<Mat> desc(2);
    fast->detect(I[0],kp[0]);
    brisk->compute(I[0],kp[0],desc[0]);

    vector<Mat> H_ret(I.size() - 1);

    for (unsigned i = 1; i < I.size(); i++) {
        Mat H = H_ret[i-1];
        if (H_load.size() > 0) {
            H = H_load[i-1];
        } else if (H_ret[i-1].empty()) {
            fast->detect(I[i],kp[1]);
            brisk->compute(I[i],kp[1],desc[1]);
            vector<Mat> H_ = findHomographies(1, 1, desc, kp);
            H = H_[0];
        }
        // "Perspective transform errors"
        Mat I_back, concat;
        if (H.empty()){
            std::cerr << "Transform not found!" << endl;
            I_back = Mat(I[0].size(), I[0].depth(), cv::Scalar(0));
            H = Mat::eye(3,3, CV_64F);
        }
            
        cv::warpPerspective(I[i], I_back, H.inv(), I[i].size());

        vector<Mat> v = {I[0],I[i],I_back};
        for (auto& im : v)
            if (im.channels() == 1)
                cv::cvtColor(im,im, cv::COLOR_GRAY2BGR);
        
        Mat I_diff(I[0].size(),CV_64F);
        I_diff = I[0] - I_back;
        I_diff.setTo(0,I_diff ==  I[0]);
        cv::normalize(I_diff,I_diff,0,255,cv::NORM_MINMAX);
        I_diff.convertTo(I_diff,I[0].depth());
        
        cv::applyColorMap(I_diff,I_diff,cv::COLORMAP_JET);
        v.push_back(I_diff);        
        cv::hconcat(v,concat);

        cout << "Img " << i << " / " << I.size() - 1 << ", mean difference: " << cv::mean(I_diff)[0] << endl; 

        cv::imshow("Reference | Original | Inverse transform | Difference",concat);
        
        char key = cv::waitKey(0) & 0xEFFFFF;
        if (key == 8 & i > 2) { // Backspace - prev img
            i-=2;
            continue;
        }
        if (key == 9 & i > 0) { // Tab - redo calc
            H_ret[i-1] = Mat();
            i--;
            continue;
        }
        if (key == 27) // Esc
            break;
        if (key == 32)
            H = manualSelectHomography(I[0], I[i]);
        if (key == 80 || key == 112) // p or P - set previous homography
            H = H_ret[i-2];
        // Any other key - proceed with next img
        if (H_load.size() > 0) 
            H_load[i-1] = H;
        H_ret[i-1] = H;
    }

    return H_ret;
}

vector<vector<double>> flattenMats(vector<Mat> M)
{
    vector<vector<double>> v(M.size());
    for (unsigned i = 0; i < M.size(); i++) {
        cout << M[i] << endl;
        Mat m = M[i].reshape(1,1);
        cout << m << endl;
        v[i] = m;
        cout << v[i] << endl;
    }
    return v;
}

char showWarped(Mat I, Mat M)
{
    Mat I_tr;
    cv::warpPerspective(I, I_tr, M, I.size());
    cv::imshow("Transformed image",I_tr);
    return cv::waitKey(0) & 0xEFFFFF;
}

void showWarped(vector<Mat> I, vector<Mat> M)
{
    for (unsigned i=0; i<I.size(); i++)
        showWarped(I[i],M[i]);
}

vector<Mat> readTestDataset(cv::String path)
{
    vector<vector<double>> flat_H = loadCSV(path);
    vector<Mat> H(flat_H.size());
    for (unsigned i=0; i < flat_H.size(); i++) {
        H[i] = Mat(flat_H[i]).reshape(1,3);
        H[i].convertTo(H[i], CV_32F);
    }
    return H;
}

// Generate mask to pass to keypoint detector signalling the edges of 
// the transformed frame. This prevents the detector to search for
// keypoints outside or on the edge
Mat genMask(Mat H, int rows, int cols, int type)
{
    Mat mask = Mat(rows, cols, type, cv::Scalar(0));
    int margin = 10;
    mask({margin,rows-margin},{margin,cols-margin}) = cv::Scalar(255,255,255);
    cv::warpPerspective(mask, mask, H, mask.size());
    return mask;
}

vector<Mat> genMask(vector<Mat> H, int rows, int cols, int type)
{
    vector<Mat> masks(H.size());
    #pragma omp parallel for
    for (unsigned i = 0; i < masks.size(); i++)
        masks[i] =  genMask(H[i], rows, cols, type);
    return masks;
}

// Remove homographies for reference images and multiply others by their inverse
vector<Mat> removeRefHomographies(vector<Mat> H_in, unsigned n_refs)
{
    unsigned n_gen = (H_in.size() - n_refs) / n_refs;
    vector<Mat> H(n_refs*n_gen);
    for (unsigned i = 0; i < n_refs; i++)
        for (unsigned j = 0; j < n_gen; j++)
            H[i * n_gen + j] = H_in[i * (n_gen + 1) + j + 1] * H_in[i * (n_gen + 1)].inv();
    return H;
}

std::pair<vector<vector<KeyPoint>>,vector<Mat>> calcKpDesc(vector<Mat> I, unsigned n_refs, vector<Mat> masks = {})
{
    unsigned n_gen = (I.size() - n_refs) / n_refs;
    while (I.size() > n_refs * (n_gen + 1)) {
        cout << "n_images not divisible by n_refs, popping img " << I.size() << " from dataset" << endl;
        I.pop_back();
    }

    cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(18,false,cv::FastFeatureDetector::TYPE_9_16);
    cv::Ptr<cv::BRISK> brisk = cv::BRISK::create(0,0,1.8);

    vector<vector<KeyPoint>> kp;
    vector<Mat> desc;
    if (masks.size() > 0)
        fast->detect(I,kp,masks);
    else
        fast->detect(I,kp);

    brisk->compute(I,kp,desc);
    
    return {kp,desc};
}

/*
    params.loMethod = cv::LocalOptimMethod::LOCAL_OPTIM_GC;
    params.loIterations = 8;
    params.threshold = 6;
    params.maxIterations = 6000;
    params.score = cv::ScoreMethod::SCORE_METHOD_MAGSAC;
*/

vector<cv::LocalOptimMethod> lomethods = {
    cv::LocalOptimMethod::LOCAL_OPTIM_INNER_LO,
    cv::LocalOptimMethod::LOCAL_OPTIM_INNER_AND_ITER_LO,
};

vector<cv::SamplingMethod> samplingmethods{
    cv::SamplingMethod::SAMPLING_UNIFORM,
    cv::SamplingMethod::SAMPLING_PROGRESSIVE_NAPSAC,
    cv::SamplingMethod::SAMPLING_NAPSAC,
    cv::SamplingMethod::SAMPLING_PROSAC
};

vector<cv::ScoreMethod> scoremethods {
    cv::ScoreMethod::SCORE_METHOD_MSAC,
    cv::ScoreMethod::SCORE_METHOD_MAGSAC,
};

vector<vector<double>> runTest(vector<Mat> I, vector<Mat> H_ref, unsigned n_refs, vector<Mat> masks = {})
{
    init(I[0].cols,I[0].rows);
    cv::UsacParams usacparams;
    unsigned n_gen = (I.size() - n_refs) / n_refs;
    auto kpdesc = calcKpDesc(I,n_refs,masks);
    vector<vector<double>> score;

    //for (double threshold = 3; threshold < 10; threshold+=0.2) {

    //for (unsigned losamplesize = 1; losamplesize < 50; losamplesize++) {

    for (unsigned maxiter = 5000; maxiter < 100000; maxiter+=5000) {
    vector<double> score_row;
        //for (auto scoremethod : scoremethods) {
        //for (auto lomethod : lomethods) {
            usacparams.threshold = 6;
            usacparams.sampler = cv::SamplingMethod::SAMPLING_UNIFORM;
            usacparams.loSampleSize = 12;
            usacparams.loMethod = cv::LocalOptimMethod::LOCAL_OPTIM_INNER_AND_ITER_LO;
            usacparams.score = cv::ScoreMethod::SCORE_METHOD_MSAC;
            usacparams.maxIterations = 60000;
            //cout << "Maxiters: " << << ", time by frame: "; 
            vector<Mat> H = findHomographies(n_refs, n_gen, kpdesc.second, kpdesc.first, usacparams);
            cout << " us" <<  endl;
            score_row = meanTransformError(H, H_ref, test_points);
            //double score_single = vsum(meanTransformError(H, H_ref, test_points)) / H.size();
            //cout << "maxiter: " << maxiter << ", score: " << score_single << endl;
            //out << "losamplesize: " << losamplesize << ", score: " << score_single << endl;
            //cout << "threshold: " << threshold << ", score: " << score_single << endl;
            //cout << "sampler: " << samplemethod << ", scoreMethod: " << scoremethod << ", score: " << score_single << endl;
            //cout << "loMethod: " << lomethod << ", sampler: " << samplemethod << ", score: " << score_single << endl;
            //cout << "loMethod: " << lomethod << ", losamplesize: " << losamplesize << ", score: " << score_single << endl;
            //score_row.push_back(score_single);
        //}
        score.push_back(score_row);
    }
    return score;
}


int main()
{
    //srand (time(NULL));
    srand (0);
    bool train = true, test = false;
    bool toy = false;

    cv::String testname = "usac_bestparam";
    //cv::String testname = "usac_loSampleSize";

    if (train) {
        cout << "Running train:" << endl;
        unsigned n_refs;
        vector<Mat> train_I;
        if (toy) {
            n_refs = 4;
            train_I = read_imgs("./toy_img/*.png");
        } else {
            n_refs = 100;
            train_I = read_imgs("/home/rozsatib/Dropbox/thermac/archive/imgs/outpy/*.png");
        }
        vector<Mat> train_H = generateHomographies(train_I.size(), train_I[0].cols, train_I[0].rows,false);
        train_I = add_noise(train_I, 7, 5);
        train_I = warpWithNoise(train_I, train_H);
        vector<Mat> masks = genMask(train_H, train_I[0].rows, train_I[0].cols, train_I[0].depth());

        train_H = removeRefHomographies(train_H,n_refs);

        train_I = preprocess(train_I);
        vector<vector<double>> score = runTest(train_I,train_H,n_refs,masks);
        cv::String filename = "train_" + testname + ".csv";
        //writeCSV(filename,score);
    }

    if (test) {
        cout << "Running test:" << endl;

        vector<Mat> test_I = read_vid("/home/rozsatib/Dropbox/thermac/archive/videos/usac-train-raw.avi");
        vector<Mat> test_H = readTestDataset("usac_train_H.csv");
        if (toy)
            for(int i=0; i<500; i++) {
                test_I.pop_back();
                test_H.pop_back();
            }
    
        test_I = preprocess(test_I);

        vector<vector<double>> score = runTest(test_I,test_H,1);
        cv::String filename = "test_" + testname + ".csv";
        //writeCSV(filename,score);
    }
    return 0;
}
