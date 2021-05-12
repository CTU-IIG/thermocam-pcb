#include "thermo_img.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <err.h>
#include "point-tracking.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "Base64.h"
#include <future>
#include <iostream>
#include <algorithm>

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

void thermo_img::update(img_stream &is)
{
    is.get_image(rawtemp);

    rawtemp.convertTo(gray, CV_8U,
                      255.0 / (is.max_rawtemp - is.min_rawtemp),
                      255.0 / (1.0 - double(is.max_rawtemp) / double(is.min_rawtemp)));

    this->is = &is;
    webimgs.clear();
}

// Prints vector of strings at given point in a column
// Workaround for the missing support of linebreaks from OpenCV
static void imgPrintStrings(Mat& img, cv::Ptr<cv::freetype::FreeType2> ft2,
                            vector<string> strings, Point2f p, Scalar color)
{
    int fontHeight = 15;
    int thickness = -1;
    int linestyle = cv::LINE_AA;

    int bl = 0;
    Size sz = ft2->getTextSize("A", fontHeight, thickness, &bl);
    for (string s : strings) {
        ft2->putText(img, s, p, fontHeight, color, thickness, linestyle, false);
        p.y += sz.height * 15 / 10;
    }
}

static int getSidebarWidth(vector<POI> poi)
{
    int max = 0; // max text width
    int bl = 0; // baseline
    for (unsigned i = 0; i < poi.size(); i++) {
        string s = "00: " + poi[i].name + " 000.00 C";
        Size sz = getTextSize(s, FONT_HERSHEY_COMPLEX_SMALL, 1, 2, &bl);
        max = (sz.width > max) ? sz.width : max;
    }
    return max;
}

static void drawSidebar(Mat& img, cv::Ptr<cv::freetype::FreeType2> ft2, vector<POI> poi)
{
    // Draw sidebar
    int sbw = getSidebarWidth(poi);
    copyMakeBorder(img, img, 0, 0, 0, sbw, BORDER_CONSTANT, Scalar(255, 255, 255));

    // Print point names and temperatures
    vector<string> s(poi.size());
    for (unsigned i = 0; i < poi.size(); i++)
        s[i] = to_string(i) + ": " + poi[i].to_string() + "°C";
    Point2f print_coords = { (float)(img.cols - sbw + 5), 0 };
    imgPrintStrings(img, ft2, s, print_coords, Scalar(0, 0, 0));
}

Mat drawPOI(Mat in, cv::Ptr<cv::freetype::FreeType2> ft2, vector<POI> poi, draw_mode mode)
{
    Mat R;

    R = in.clone();
    if (R.channels() == 1)
        cvtColor(R, R, COLOR_GRAY2BGR);

    resize(R, R, Size(), 2, 2); // Enlarge image 2x for better looking fonts
    if (mode == NUM)
        drawSidebar(R, ft2, poi);

    for (unsigned i = 0; i < poi.size(); i++) {
        poi[i].p *= 2; // Rescale points with image
        circle(R, poi[i].p, 3, Scalar(0,255,0), -1); // Dot at POI position

        // Print point labels
        vector<string> label;
        switch (mode) {
        case FULL:
            label = { poi[i].name, poi[i].to_string(false).append("°C") };
            break;
        case TEMP:
            label = { poi[i].to_string(false).append("°C") };
            break;
        case NUM:
            label = { to_string(i) };
            break;
        }
        imgPrintStrings(R, ft2, label, poi[i].p + Point2f(3, 6), Scalar(0,0,0));
        imgPrintStrings(R, ft2, label, poi[i].p + Point2f(2, 5), Scalar(0,255,0));
    }

    return R;
}

void thermo_img::draw_preview(draw_mode mode, cv::Ptr<cv::freetype::FreeType2> ft2)
{
    Mat img;
    gray.copyTo(img);
    applyColorMap(img, img, cv::COLORMAP_INFERNO);
    img = drawPOI(img, ft2, poi, mode);

    for(unsigned i = 0; i < heat_sources_border.size(); i++){
        line(img, heat_sources_border[i] * 2, heat_sources_border[(i + 1) % heat_sources_border.size()] * 2,
                Scalar(0,0,255));
    }
    preview = img;
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

void thermo_img::read_from_poi_json(string poi_filename, string heat_sources_border_points)
{
    gray = readJsonImg(poi_filename);
    poi = readPOI(poi_filename);
    rawtemp.create(gray.size()); // to make width() and height() return expected values

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

void thermo_img::write_poi_json(string path, bool verbose)
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

void thermo_img::add_poi(POI &&p)
{
    poi.push_back(p);
}

void thermo_img::pop_poi()
{
    poi.pop_back();
}

void thermo_img::track(const thermo_img &ref, tracking track)
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
        static future<thermo_img> future;

        if (!future.valid() ||
            future.wait_for(chrono::seconds::zero()) == future_status::ready) {
            if (future.valid()) {
                thermo_img tracked = future.get();
                poi = tracked.poi;
                heat_sources_border = tracked.heat_sources_border;
            }

            future = async([&](thermo_img copy) {
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

void thermo_img::updateKpDesc()
{
    Mat pre = preprocess(gray);
    nc.kp = getKeyPoints(pre);
    nc.desc = getDescriptors(pre, nc.kp);
}

void thermo_img::trainMatcher()
{
    updateKpDesc();
    ::trainMatcher(nc.desc);
}

double thermo_img::get_temperature(uint16_t pixel)
{
    return is->get_temperature(pixel);
}


double thermo_img::get_temperature(Point p)
{
    if (p.y < 0 || p.y > height() || p.x < 0 || p.x > width()) {
        cerr << "Point at (" << p.x << "," << p.y << ") out of image!" << endl;
        return nan("");
    }
    if (!is)
        return nan("");

    uint16_t pixel = rawtemp(p);
    return is->get_temperature(pixel);
}

void thermo_img::updatePOICoords(const thermo_img &ref)
{
    std::vector<cv::DMatch> matches = matchToReference(nc.desc);
    Mat H = findH(ref.nc.kp, nc.kp, matches);

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

void thermo_img::calcHeatSources()
{
    for (auto &p : heat_sources_border) {
        if (p.x < 0 || p.x > width() || p.y < 0 || p.y > height()) {
            cerr << "Heat source border out of the image!" << endl;
            return;
        }
    }

    const Size sz(100, 100); // Size of heat sources image
    vector<Point2f> detail_rect = { {0, 0}, {float(sz.width), 0}, {float(sz.width), float(sz.height)}, {0, float(sz.height)} };

    Mat transform = getPerspectiveTransform(heat_sources_border, detail_rect);
    Mat raw_float, detail;
    rawtemp.convertTo(raw_float, CV_64F);
    warpPerspective(raw_float, detail, transform, sz);
    {
        double min, max;
        stringstream ss;

        minMaxLoc(detail, &min, &max);
        ss << fixed << setprecision(2) << get_temperature(max) << "–" << get_temperature(min) << "=" <<
            get_temperature(max) - get_temperature(min) << "°C";
        webimgs.emplace_back("detail-current", "Detail", detail, ss.str());
    }


    Mat blur, laplacian, hsImg;
    const double blur_sigma = 7;
    GaussianBlur(detail, blur, Size(0, 0), blur_sigma, blur_sigma);
    Laplacian(blur, laplacian, blur.depth());
    laplacian *= -1;
    double lap_max = 0;
    minMaxLoc(laplacian, nullptr, &lap_max);
    webimgs.emplace_back("laplacian-current", "Laplacian", laplacian, "max: " + to_string(lap_max));

    vector<Point> lm = localMaxima(laplacian, hsImg);
    webimgs.emplace_back("heat_sources-current", "Heat sources", hsImg);

    static array<Mat, 3> hsAvg;
    for (auto [i, alpha] : { make_pair(0U, 0.9), {1, 0.99}, {2, 0.999} }) {
        if (hsAvg[i].empty())
            hsAvg[i] = hsImg * 0.0; // black image of the same type and size
        hsAvg[i] = alpha * hsAvg[i] + (1-alpha) * hsImg;

        Mat hs_log;
        // Make the dark colors more visible
        cv::log(0.001+hsAvg[i], hs_log);
        //cv::sqrt(hsAvg_out, hsAvg_out);
        webimgs.emplace_back("hs-avg" + to_string(i), "HS avg. α=" + to_string(alpha), hs_log);
    }

    const auto offset = 0.025;
    Mat lapgz;
    Mat lapx = laplacian - offset;
    lapx.copyTo(lapgz, lapx > 0.0);
    webimgs.emplace_back("lapgz", "Lapl. > " + to_string(offset), lapgz);

    static array<Mat, 2> lapgz_avg;
    for (auto [i, alpha] : { make_pair(0U, 0.9), {1, 0.99} }) {
        if (lapgz_avg[i].empty())
            lapgz_avg[i] = lapgz * 0.0; // black image of the same type and size
        lapgz_avg[i] = alpha * lapgz_avg[i] + (1-alpha) * lapgz;
        webimgs.emplace_back("lapgz-avg" + to_string(i), "L> avg. α=" + to_string(alpha), lapgz_avg[i]);
    }

    hs.resize(lm.size());
    size_t max_hs = 0;
    for (unsigned i=0; i<lm.size(); i++) {
        hs[i].location = lm[i];
        hs[i].temperature = get_temperature(detail.at<double>(lm[i]));
        hs[i].neg_laplacian = laplacian.at<double>(lm[i]);
        if (hs[i].neg_laplacian > hs[max_hs].neg_laplacian)
            max_hs = i;
    }
}

const std::vector<cv::Point2f> &thermo_img::get_heat_sources_border() const
{
    return heat_sources_border;
}

const std::vector<POI> &thermo_img::get_poi() const
{
    return poi;
}

cv::Mat thermo_img::get_gray() const
{
    return gray;
}

int thermo_img::height() const
{
    return rawtemp.rows;
}

int thermo_img::width() const
{
    return rawtemp.cols;
}

const std::list<thermo_img::webimg> &thermo_img::get_webimgs() const
{
    return webimgs;
}

const thermo_img::webimg *thermo_img::get_webimg(string key) const
{
    auto it = find_if(webimgs.begin(), webimgs.end(), [&](const webimg &wi){return wi.name == key;});
    return (it != webimgs.end()) ? &(*it) : nullptr;
}

const Mat thermo_img::get_rgb(string key) const
{
    const webimg *si = get_webimg(key);
    return si ? si->rgb : Mat();
}

const cv::Mat thermo_img::get_detail() const
{
    return get_rgb("detail-current");
}

const cv::Mat thermo_img::get_laplacian() const
{
    return get_rgb("laplacian-current");
}

const cv::Mat thermo_img::get_hs_img() const
{
    return get_rgb("heat_sources-current");
}

const cv::Mat thermo_img::get_hs_avg() const
{
    return get_rgb("hs-avg0");
}

const std::vector<HeatSource> &thermo_img::get_heat_sources() const
{
    return hs;
}

const cv::Mat &thermo_img::get_preview() const
{
    return preview;
}

cv::Mat_<uint16_t> thermo_img::get_rawtemp() const
{
    return rawtemp;
}

thermo_img::webimg::webimg(string name, string title, const Mat &mat, string desc)
    : name(name)
    , title(title)
    , mat(mat)
    , rgb(normalize(mat))
    , html_desc(desc)
{}

Mat thermo_img::webimg::normalize(Mat in)
{
    Mat out;
    normalize_and_convert_to_uchar(in, out);
    applyColorMap(out, out, cv::COLORMAP_INFERNO);
    resize(out, out, Size(), 2, 2);
    return out;
}
