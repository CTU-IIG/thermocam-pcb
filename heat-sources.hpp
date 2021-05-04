#pragma once

#include "img_stream.hpp"
#include "thermo_img.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/freetype.hpp>

double getTemp(cv::Point p, img_stream &is, im_status &s);

struct HeatSource {
    cv::Point location;
    double temperature;
    double neg_laplacian;
};

std::vector<HeatSource> heatSources(im_status &s, cv::Mat &laplacian, cv::Mat &hsImg, cv::Mat &detail,
                                    array<cv::Mat, 3> &hsAvg_out, cv::Ptr<cv::freetype::FreeType2> ft2);
