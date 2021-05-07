#pragma once

#include "img_stream.hpp"
#include "thermo_img.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/freetype.hpp>

double getTemp(cv::Point p, img_stream &is, im_status &s);
