#ifndef POINT_TRACKING_H
#define POINT_TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>

std::vector<cv::KeyPoint> getKeyPoints(cv::Mat A);
cv::Mat getDescriptors(cv::Mat A, std::vector<cv::KeyPoint>& kp);
cv::Mat findHomography(std::vector<cv::KeyPoint> kp_from,
                       std::vector<cv::KeyPoint> kp_to, cv::Mat desc_from,
                       cv::Mat desc_to);
cv::Mat preprocess(cv::Mat input);

#endif
