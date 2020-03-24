#include "point_tracking.h"
#include <iostream>

std::vector<cv::KeyPoint> getKeyPoints(cv::Mat A) {
    std::vector<cv::KeyPoint> kp;
    cv::FastFeatureDetector fast(40, true);
    //cv::FastFeatureDetector fast(40);
    fast.detect(A, kp);
    return kp;
}

cv::Mat getDescriptors(cv::Mat A, std::vector<cv::KeyPoint>& kp)
{
    cv::Mat desc;
    cv::Ptr<cv::DescriptorExtractor> de = cv::FeatureDetector::create("ORB");
    de->compute(A, kp, desc);
    return desc;
}

cv::Mat findHomography(std::vector<cv::KeyPoint> kp_from, std::vector<cv::KeyPoint> kp_to, cv::Mat desc_from, cv::Mat desc_to)
{
    // Convert descriptors to float form
    desc_to.convertTo(desc_to, CV_32F);
    desc_from.convertTo(desc_from, CV_32F);

    // FLANN keypoint matching
    std::vector<std::vector<cv::DMatch>> matches;
    cv::FlannBasedMatcher matcher;
    matcher.knnMatch(desc_to, desc_from, matches, 2);
    sort(matches.begin(), matches.end());

    if (0) { 
        std::cout << "from kp: " << kp_from.size() << ", to kp: " << kp_to.size() << std::endl;
        std::cout << "matches size: " << matches.size() << std::endl;
    }

    // Select good matches
    double thresh = 0.9;
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < thresh * matches[i][1].distance)
            good_matches.push_back(matches[i][0]);
    }
    
    if (good_matches.size() < 4) // Cannot calculate homography
        return cv::Mat();
    
    std::vector<cv::Point2f> toP, fromP;
    for (size_t i = 0; i < good_matches.size(); i++) {
        toP.push_back(kp_to[good_matches[i].queryIdx].pt);
        fromP.push_back(kp_from[good_matches[i].trainIdx].pt);
    }
    
    if (0)
        std::cout << "RANSAC input size:" << toP.size() << std::endl;
    return findHomography(fromP, toP, CV_RANSAC, 1);
}

cv::Mat preprocess(cv::Mat input)
{
    cv::Mat im = input.clone();
    // Median filter + contrast limited histogram equalization
    cv::medianBlur(im, im, 3);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(12, cv::Size(32, 32));
    clahe->apply(im, im);
    // Image sharpening by unsharp masking with gaussian blur
    cv::Mat sh;
    cv::GaussianBlur(im, sh, cv::Size(0, 0), 3, 3);
    cv::addWeighted(im, 1.5, sh, -0.5, 0, sh);
    return im;
}
