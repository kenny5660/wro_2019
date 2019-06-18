#pragma once
#include <opencv2/core.hpp>
#include "logic_structures.h"

color_t VisionGetSmallBox(const cv::Mat& frame);

color_t VisionGetBigBox(const cv::Mat& frame, double dist);