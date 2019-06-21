#pragma once
#include <opencv2/core.hpp>
#include "logic_structures.h"
#if !(defined(WIN32) || defined(_WIN32) || \
      defined(__WIN32) && !defined(__CYGWIN__))
#include <Robot.h>
#else
namespace Robot {
enum class CatchCubeSideEnum { LEFT, RIGHT, NONE };
}
#endif
color_t VisionGetSmallBox(const cv::Mat& frame, Robot::CatchCubeSideEnum side);
color_t VisionGetBigBox(const cv::Mat& frame, double dist);
std::string color_t2str(color_t color);
