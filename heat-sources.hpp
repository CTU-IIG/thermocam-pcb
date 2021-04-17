#include "img_stream.hpp"
#include "thermo_img.hpp"
#include <opencv2/core/mat.hpp>

double getTemp(cv::Point p, img_stream &is, im_status &s);

std::vector<POI> heatSources(im_status &s, cv::Mat &laplacian, cv::Mat &hsImg, cv::Mat &detail);
