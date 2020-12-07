#include "point-tracking.hpp"
#include <err.h>
#include <iostream>

// knnMatchImpl is protected in FlannBasedMatcher, only knnMatch is public.
// knnMatch roughly only runs add(), train(), and knnMatchImpl()
// That means we would have to train the matcher, thus constructing the
// Kd tree used for approximate nearest neighbour every time from the same
// reference image, which is useless.
// Thus, this ugly hack, to be able to use knnMatchImpl and train only once.
class MyFlann : public cv::FlannBasedMatcher {
public:
    void knnMatchImpl(const cv::Mat& queryDescriptors, std::vector<std::vector<cv::DMatch>>& matches, int knn) {cv::FlannBasedMatcher::knnMatchImpl(queryDescriptors,matches,knn);}
};

MyFlann matcher;
cv::Ptr<cv::FastFeatureDetector> fast = nullptr;
cv::Ptr<cv::BRISK> brisk = nullptr;

std::vector<cv::KeyPoint> getKeyPoints(cv::Mat A) {
    if (!fast) fast = cv::FastFeatureDetector::create(18, false);
    std::vector<cv::KeyPoint> kp;
    fast->detect(A, kp);
    return kp;
}

// getDescriptors changes the KeyPoint vector, as it removes keypoints which are
// impossible to calculate a keypoint from (too close to image border, etc.).
// Thus kp should always be passed as a non-const reference.
cv::Mat getDescriptors(cv::Mat A, std::vector<cv::KeyPoint>& kp)
{
    cv::Mat desc;
    if (!brisk) brisk = cv::BRISK::create(0,0,1.8);
    brisk->compute(A, kp, desc);
    return desc;
}

void trainMatcher(cv::Mat desc_train)
{
    desc_train.convertTo(desc_train, CV_32F);
    matcher.add({desc_train});
    matcher.train();
}

std::vector<cv::DMatch> matchToReference(cv::Mat desc_query)
{
    std::vector<std::vector<cv::DMatch>> matches;
    cv::Mat _desc_query;
    desc_query.convertTo(_desc_query, CV_32F);
    if(matcher.empty())
        err(1,"Cannot match, matcher not trained with reference image!");
    matcher.knnMatchImpl(_desc_query,matches,2);

    // Select good matches
    double thresh = 0.88;
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < thresh * matches[i][1].distance)
            good_matches.push_back(matches[i][0]);
    }
    return good_matches;
}

cv::Mat findH(const std::vector<cv::KeyPoint> &kp_from,
              const std::vector<cv::KeyPoint> &kp_to,
              const std::vector<cv::DMatch> &matches)
{
    // FLANN keypoint matching
    if (matches.size() < 4) // Cannot calculate homography
        return cv::Mat();

    std::vector<cv::Point2f> toP(matches.size()), fromP(matches.size());
    for (size_t i = 0; i < matches.size(); i++) {
        toP[i] = kp_to[matches[i].queryIdx].pt;
        fromP[i] = kp_from[matches[i].trainIdx].pt;
    }

    cv::UsacParams params;
    params.threshold = 6;
    params.sampler = cv::SamplingMethod::SAMPLING_UNIFORM;
    params.loSampleSize = 12;
    params.loMethod = cv::LocalOptimMethod::LOCAL_OPTIM_INNER_AND_ITER_LO;
    params.score = cv::ScoreMethod::SCORE_METHOD_MSAC;
    // Ridiculously high required confidence, so maxIterations is always reached
    // for more deterministic runtime
    params.confidence = 0.9999999999999999999;
    params.maxIterations = 90000;
    return findHomography(fromP, toP, cv::noArray(), params);
}

cv::Mat preprocess(cv::Mat input)
{
    cv::Mat im = input.clone();
    // Median filter + contrast limited histogram equalization
    cv::medianBlur(im, im, 3);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(18, cv::Size(8, 8));
    clahe->apply(im, im);
    // Image sharpening by unsharp masking with gaussian blur
    cv::Mat sh;
    cv::GaussianBlur(im, sh, cv::Size(0, 0), 7, 7);
    cv::addWeighted(im, 1.5, sh, -0.5, 0, sh);
    return im;
}
