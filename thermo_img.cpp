#include "thermo_img.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <err.h>
#include "point-tracking.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "Base64.h"
#include <future>

using namespace std;
using namespace cv;
namespace pt = boost::property_tree;

string POI::to_string(bool print_name)
{
    stringstream ss;
    if (print_name)
        ss << name << "=";
    ss << fixed << setprecision(2) << temp;
    return ss.str();
}

void im_status::update(img_stream &is)
{
    is.get_image(rawtemp);
    width = rawtemp.cols;
    height = rawtemp.rows;

    rawtemp.convertTo(gray, CV_8U,
                      255.0 / (is.max_rawtemp - is.min_rawtemp),
                      255.0 / (1.0 - double(is.max_rawtemp) / double(is.min_rawtemp)));

    this->is = &is;
}

static vector<POI> readPOI(string path)
{
    vector<POI> poi;
    pt::ptree root;
    pt::read_json(path, root);
    for (pt::ptree::value_type &p : root.get_child("POI"))
        poi.push_back({ p.second.get<string>("name"),
                        { p.second.get<float>("x"), p.second.get<float>("y") },
                        p.second.get<double>("temp")});
    return poi;
}

static Mat readJsonImg(string path)
{
    pt::ptree root;
    pt::read_json(path, root);
    pt::ptree img_ptree = root.get_child("POI img");
    string img_encoded =  img_ptree.get_value<string>();
    string decoded;
    if(macaron::Base64::Decode(img_encoded,decoded) != "")
        err(1,"Base64 decoding: input data size is not a multiple of 4.");
    vector<uchar> decoded_v(decoded.begin(), decoded.end());
    return imdecode(decoded_v,0);
}

vector<string> split(const string str, const char *delimiters)
{
    vector<string> words;

    for (size_t start = 0, end = 0;;) {
        start = str.find_first_not_of(delimiters, end);
        if (start == string::npos)
            break;

        end = str.find_first_of(delimiters, start);
        if (end == string::npos)
            end = str.size();

        words.push_back(str.substr(start, end - start));
    }
    return words;
}

void im_status::read_from_poi_json(string poi_filename, string heat_sources_border_points)
{
    gray = readJsonImg(poi_filename);
    poi = readPOI(poi_filename);
    width = gray.cols;
    height = gray.rows;

    if (!heat_sources_border_points.empty()) {
        vector<string> pt_names = split(heat_sources_border_points, ",");
        if (pt_names.size() != 4)
            throw runtime_error("Four heat source point names are required, not " + to_string(pt_names.size()) +
                                " as in: " + heat_sources_border_points);
        for (auto &name: pt_names) {
            POI *p = nullptr;
            for (auto &poi : this->poi) {
                if (poi.name == name) {
                    p = &poi;
                    break;
                }
            }
            if (!p)
                throw runtime_error("Heat source point '" + name + "' not found in " + poi_filename);
            heat_sources_border.push_back(p->p);
            poi.erase(remove_if(poi.begin(), poi.end(), [p](POI &pp){return &pp == p;}),
                      poi.end());
        }
    }
}

void im_status::write_poi_json(string path, bool verbose)
{
    pt::ptree root, poi_pt, poi_img;
    for (unsigned i = 0; i < poi.size(); i++) {
        pt::ptree elem;
        elem.put("name", poi[i].name);
        elem.put("x", poi[i].p.x);
        elem.put("y", poi[i].p.y);
        elem.put("temp", poi[i].temp);
        poi_pt.push_back(std::make_pair("", elem));
    }
    root.add_child("POI", poi_pt);

    vector<uchar> img_v;
    imencode(".jpg",gray,img_v);
    string img_s(img_v.begin(),img_v.end());
    poi_img.put("", macaron::Base64::Encode(img_s));
    root.add_child("POI img", poi_img);

    pt::write_json(path, root);

    if (verbose)
        cout << "Points saved to " << path << endl;
}

void im_status::add_poi(POI &&p)
{
    poi.push_back(p);
}

void im_status::pop_poi()
{
    poi.pop_back();
}

void im_status::track(const im_status &ref, tracking track)
{
    switch (track) {
    case tracking::off:
        break;
    case tracking::sync:
        updateKpDesc();
        updatePOICoords(ref);
        break;
    case tracking::async:
        // TODO: Remove static - make it a member variable
        static future<im_status> future;

        if (!future.valid() ||
            future.wait_for(chrono::seconds::zero()) == future_status::ready) {
            if (future.valid()) {
                im_status tracked = future.get();
                poi = tracked.poi;
                heat_sources_border = tracked.heat_sources_border;
            }

            future = async([&](im_status copy) {
                copy.updateKpDesc();
                copy.updatePOICoords(ref);
                return copy;
            }, *this);
        }
        break;
    case tracking::finish:
        future.wait();
        break;
    }

    for (POI& point : poi)
        point.temp = get_temperature((Point)point.p);
}

void im_status::updateKpDesc()
{
    Mat pre = preprocess(gray);
    kp = getKeyPoints(pre);
    desc = getDescriptors(pre, kp);
}

void im_status::trainMatcher()
{
    ::trainMatcher(desc);
}

double im_status::get_temperature(uint16_t pixel)
{
    return is->get_temperature(pixel);
}


double im_status::get_temperature(Point p)
{
    if (p.y < 0 || p.y > height || p.x < 0 || p.x > width) {
        cerr << "Point at (" << p.x << "," << p.y << ") out of image!" << endl;
        return nan("");
    }
    if (!is)
        return nan("");

    uint16_t pixel = rawtemp(p);
    return is->get_temperature(pixel);
}

void im_status::updatePOICoords(const im_status &ref)
{
    std::vector<cv::DMatch> matches = matchToReference(desc);
    Mat H = findH(ref.kp, kp, matches);

    if (H.empty()) // Couldn't find homography - points stay the same
        return; // FIXME: Let the caller (or at least user) know that this happened

    if (poi.size() != ref.poi.size())
        poi = ref.poi;

    for (unsigned i=0; i < poi.size(); i++) {
        vector<Point2f> v = { ref.poi[i].p };
        perspectiveTransform(v, v, H); // only takes vector of points as input
        poi[i].p = v[0];

        // Variance of sum of 2 random variables the same as sum of variances
        // So we only need to track 1 variance per point
        poi[i].r_var(poi[i].p.x + poi[i].p.y);
        namespace acc = boost::accumulators;
        poi[i].rolling_std = sqrt(acc::rolling_variance(poi[i].r_var));
    }

    if (ref.heat_sources_border.size() > 0)
        perspectiveTransform(ref.heat_sources_border, heat_sources_border, H);
}

const std::vector<cv::Point2f> &im_status::get_heat_sources_border() const
{
    return heat_sources_border;
}

const std::vector<POI> &im_status::get_poi() const
{
    return poi;
}
