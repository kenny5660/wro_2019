#pragma once
#include <opencv2/core.hpp>
#include "map.h"

box_color_t VisionGetSmallBox(const cv::Mat& frame);

box_color_t VisionGetBigBox(const cv::Mat& frame);