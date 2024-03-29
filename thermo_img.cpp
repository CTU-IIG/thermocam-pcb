#include "thermo_img.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <err.h>
#include "point-tracking.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/accumulators/statistics/rolling_count.hpp>
#include "Base64.h"
#include <iostream>
#include <algorithm>

using namespace std;
using namespace cv;
namespace pt = boost::property_tree;
namespace acc = boost::accumulators;

string POI::to_string(bool print_name)
{
    stringstream ss;
    if (print_name)
        ss << name << "=";
    ss << fixed << setprecision(2) << temp;
    return ss.str();
}

thermo_img::thermo_img(cv::Mat_<double> compenzation_img)
    : compenzation_img(compenzation_img)
{
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

static void normalize_and_convert_to_uchar(Mat &mat_in, Mat &mat_out){
    double min, max;
    minMaxLoc(mat_in, &min, &max);
    mat_in.convertTo(mat_out, CV_8UC1,
                     255.0 / (max - min),
                     255.0 / (1.0 - max / min));
}

void thermo_img::draw_preview(draw_mode mode, cv::Ptr<cv::freetype::FreeType2> ft2)
{
    Mat img;
    normalize_and_convert_to_uchar(rawtemp, img);
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

    // Copied from img_stream.cpp. FIXME: We should implement this more generically.
    uint16_t min_rawtemp = 7231;
    uint16_t max_rawtemp = 9799;

    gray.convertTo(rawtemp, CV_16U,
                   (max_rawtemp - min_rawtemp)/256.0,
                   min_rawtemp);


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
    double min, max;
    minMaxLoc(rawtemp, &min, &max);
    if (is) {
        double temp_diff = is->get_temperature(max) - is->get_temperature(min);

        // Switch tracking off when there is too small temperature
        // difference. Tracking does not work well and the tracked
        // points jump wildly everywhere.
        if (temp_diff < 7.0)
            track = poi.empty() ? tracking::copy : tracking::off;
    }

    switch (track) {
    case tracking::off:
        break;
    case tracking::copy:
        poi = ref.poi; // just copy the reference points
        heat_sources_border = ref.heat_sources_border;
        break;
    case tracking::sync:
        updateKpDesc();
        updatePOICoords(ref);
        break;
    case tracking::async:
        if (!nc.future.valid() ||
            nc.future.wait_for(chrono::seconds::zero()) == future_status::ready) {
            if (nc.future.valid()) {
                thermo_img tracked = nc.future.get();
                poi = tracked.poi;
                heat_sources_border = tracked.heat_sources_border;
            }

            nc.future = async([&](thermo_img copy) {
                copy.updateKpDesc();
                copy.updatePOICoords(ref);
                return copy;
            }, *this);
        }
        break;
    case tracking::finish:
        nc.future.wait();
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
    if (p.y < 0 || p.y >= height() || p.x < 0 || p.x >= width()) {
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
        poi[i].rolling_std = sqrt(acc::rolling_variance(poi[i].r_var));
    }

    if (ref.heat_sources_border.size() > 0)
        perspectiveTransform(ref.heat_sources_border, heat_sources_border, H);

    if (poi.size() > 0 && poi[0].rolling_std > 10) {
        // Tracking is significantly unstable => just copy the reference points
        for (size_t i = 0; i < poi.size(); i++)
            poi[i].p = ref.poi[i].p;
        heat_sources_border = ref.heat_sources_border;
        // TODO: Mark the image somehow so that users understand, why tracking doesn't work
    }
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


// to string no trailing zeros
std::string to_string_ntz(double v)
{
    string str = to_string(v);
    str.erase(str.find_last_not_of('0') + 1, string::npos);
    return str;
}

std::string to_string_prec(double v, unsigned prec)
{
    stringstream ss;
    ss << fixed << setprecision(prec) << v;
    return ss.str();
}

static double get_max(const Mat &mat) {
        double min, max;
        minMaxLoc(mat, &min, &max);
        return max;
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

    if (!compenzation_img.empty()) {
        raw_float -= compenzation_img;
    }

    warpPerspective(raw_float, detail, transform, sz);
    {
        double min, max;
        stringstream ss;

        minMaxLoc(detail, &min, &max);
        ss << fixed << setprecision(2) << get_temperature(max) << "–" << get_temperature(min) << "=" <<
            get_temperature(max) - get_temperature(min) << "°C";

        list<webimg> detail_list {webimg("detail-current", "Detail", detail, ss.str())};

        nc.det_var(MatAutoInit(detail)); // We have to convert the class to enforce use of overloaded operators
        Mat det_stddev;
        cv::sqrt(acc::rolling_variance(nc.det_var), det_stddev);

        detail_list.emplace_back("detail-stddev", "Det. stddev n="+to_string(acc::rolling_count(nc.det_var)), det_stddev);


        for (auto [i, alpha] : { make_pair(0U, 0.9), {1, 0.99}, {2, 0.997} }) {
            nc.detail_avg[i] = alpha * nc.detail_avg[i] + (1-alpha) * detail;
            minMaxLoc(nc.detail_avg[i], &min, &max);
            ss.str(""s);
            ss << fixed << setprecision(2) << get_temperature(max) << "–" << get_temperature(min) << "=" <<
                get_temperature(max) - get_temperature(min) << "°C";
            detail_list.emplace_back("detail-avg" + to_string(i), "D. avg"+to_string(i)+" α=" + to_string_ntz(alpha),
                                     nc.detail_avg[i], ss.str());
        }

        {
            const double alpha = 0.997;
            if (raw_avg.empty())
                raw_float.copyTo(raw_avg);
            else
                raw_avg = alpha * raw_avg + (1-alpha) * raw_float;

            detail_list.emplace_back("raw-avg", "raw avg. α=" + to_string_ntz(alpha), raw_avg);
        }

        webimgs.emplace_back(detail_list);
    }



    Mat blur, laplacian, hsImg;
    const double blur_sigma = 6;
    GaussianBlur(detail, blur, Size(0, 0), blur_sigma, blur_sigma);
    Laplacian(blur, laplacian, blur.depth());
    laplacian *= -1;
    double lap_max = get_max(laplacian);
    list<webimg> lapl_list { webimg("laplacian-current", "Laplacian", laplacian, "max: " + to_string_prec(lap_max, 3),
                                    webimg::PosNegColorMap::scale_max) };
    nc.lap_var(MatAutoInit(laplacian)); // We have to convert the class to enforce use of overloaded operators
    Mat lap_stddev;
    cv::sqrt(acc::rolling_variance(nc.lap_var), lap_stddev);
    lapl_list.emplace_back("laplacian-stddev", "Lap. stddev n="+to_string(acc::rolling_count(nc.lap_var)), lap_stddev);

    for (auto [i, alpha] : { make_pair(0U, 0.9), {1, 0.99}, {2, 0.997} }) {
        nc.lapl_avg[i] = alpha * nc.lapl_avg[i] + (1-alpha) * laplacian;
        lapl_list.emplace_back("lapl-avg" + to_string(i), "∇²avg"+to_string(i)+" α=" + to_string_ntz(alpha), nc.lapl_avg[i],
                               "max: " + to_string_prec(get_max(nc.lapl_avg[i]), 3),
                               webimg::PosNegColorMap::scale_max);
    }
    webimgs.push_back(lapl_list);


    vector<Point> lm = localMaxima(laplacian, hsImg);

    list<webimg> hs_list { webimg("heat_sources-current", "Heat sources", hsImg) };

    for (auto [i, alpha] : { make_pair(0U, 0.9), {1, 0.99}, {2, 0.999} }) {
        nc.hsAvg[i] = alpha * nc.hsAvg[i] + (1-alpha) * hsImg;

        Mat hs_log;
        // Make the dark colors more visible
        cv::log(0.001+nc.hsAvg[i], hs_log);
        //cv::sqrt(nc.hsAvg[i], hs_log);
        hs_list.emplace_back("hs-avg" + to_string(i), "HS avg. α=" + to_string_ntz(alpha), hs_log);
    }

    webimgs.push_back(hs_list);

    const auto offset = 0.025;
    Mat lapgz;
    Mat lapx = laplacian - offset;
    lapx.copyTo(lapgz, lapx > 0.0);
    list<webimg> lapgz_list { webimg("lapgz", "L⁺ = Lapl. > " + to_string_ntz(offset), lapgz,
                                     "max: " + to_string_prec(get_max(lapgz), 3)) };

    for (auto [i, alpha] : { make_pair(0U, 0.9), {1, 0.99}, {2, 0.997} }) {
        nc.lapgz_avg[i] = alpha * nc.lapgz_avg[i] + (1-alpha) * lapgz;
        lapgz_list.emplace_back("lapgz-avg" + to_string(i), "L⁺avg"+to_string(i)+" α=" + to_string_ntz(alpha), nc.lapgz_avg[i],
                                "max: " + to_string_prec(get_max(nc.lapgz_avg[i]), 3));
    }

    nc.hs_acc(lapgz);
    lapgz_list.emplace_back("lapgz-mean", "L⁺ mean n=1000", acc::rolling_mean(nc.hs_acc),
                         "max: " + to_string_prec(get_max(lapgz), 3));

    webimgs.push_back(lapgz_list);

    list<webimg> diff_list;

    Mat diff;
    double dmin, dmax;

    diff = nc.lapgz_avg[0] - nc.lapgz_avg[1];
    minMaxLoc(diff, &dmin, &dmax);
    diff_list.emplace_back("lapl-diff", "L⁺avg0 – L⁺avg1", diff,
                           "max: " + to_string_prec(dmax, 3) + ", " +
                           "min: " + to_string_prec(dmin, 3),
                           webimg::PosNegColorMap::scale_both);

    for (unsigned i : {1, 2}) {
        diff = nc.lapl_avg[i-1] - nc.lapl_avg[i];
        minMaxLoc(diff, &dmin, &dmax);
        diff_list.emplace_back("fulllapl-diff" + ((i > 1) ? to_string(i-1) : ""),
                               "∇²avg" + to_string(i-1) + " – ∇²avg" + to_string(i), diff,
                               "max: " + to_string_prec(dmax, 3) + ", " +
                               "min: " + to_string_prec(dmin, 3),
                               webimg::PosNegColorMap::scale_both);
    }
    webimgs.push_back(diff_list);
//     {
//         int i=0;
//         list<webimg> lap_list;
//         for (const Mat &detail : nc.detail_avg)
//         {
//             Mat blur, laplacian;
//             const double blur_sigma = 4;
//             GaussianBlur(detail, blur, Size(0, 0), blur_sigma, blur_sigma);
//             Laplacian(blur, laplacian, blur.depth());
//             laplacian *= -1;
//             double lap_max = get_max(laplacian);
//             lap_list.emplace_back("lap-det-avg"+to_string(i), "∇²(D.avg"+to_string(i)+")", laplacian, "max: " + to_string_prec(lap_max, 3),
//                                   webimg::PosNegColorMap::scale_max);
//             i++;
//         }
//         webimgs.push_back(lap_list);
//     }

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

const std::list<std::list<thermo_img::webimg>> &thermo_img::get_webimgs() const
{
    return webimgs;
}

const thermo_img::webimg *thermo_img::get_webimg(string key) const
{
    std::list<webimg>::const_iterator it;
    for (const auto &lst : webimgs) {
        it = find_if(lst.begin(), lst.end(), [&](const webimg &wi){return wi.name == key;});
        if (it != lst.end())
            return &(*it);
    }
    return nullptr;
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

template <typename CM>
thermo_img::webimg::webimg(string name, string title, const Mat &mat, string desc, CM cmap)
    : name(name)
    , title(title)
    , mat(mat)
    , rgb(normalize(mat, cmap))
    , html_desc(desc)
{
    double min, max;
    minMaxLoc(mat, &min, &max);
    if (html_desc.empty())      // default desc
        html_desc = "max: " + to_string_prec(max, 3) + ", min: " + to_string_prec(min, 3);
}

Mat thermo_img::webimg::normalize(Mat in, enum ColormapTypes cmap)
{
    Mat out;
    normalize_and_convert_to_uchar(in, out);
    applyColorMap(out, out, cmap);
    return out;
}

Mat thermo_img::webimg::normalize(Mat mat, thermo_img::webimg::PosNegColorMap pn)
{
    double min, max;
    minMaxLoc(mat, &min, &max);
    double scale = (pn == PosNegColorMap::scale_both) ?
                std::max(max, -min) : max;

    Mat pos, neg, out;
    mat.copyTo(pos, mat > 0);
    mat.copyTo(neg, mat < 0);

    Mat pos8, neg8;
    for (auto [in, out, scl, cmap] : {
            make_tuple(&pos, &pos8, +scale, COLORMAP_INFERNO),
            make_tuple(&neg, &neg8, -scale, COLORMAP_OCEAN) }) {
        in->convertTo(*out, CV_8UC1, 255.0 / scl, 0);
        applyColorMap(*out, *out, cmap);
    }
    return pos8 + neg8;
}
