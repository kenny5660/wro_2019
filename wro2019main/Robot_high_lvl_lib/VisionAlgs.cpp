#include "VisionAlgs.h"
#include "debug.h"
#include <opencv2/imgproc.hpp>
box_color_t VisionGetSmallBox(const cv::Mat& frame)
{
  cv::Rect cut_rect(cv::Point(400, 400) ,cv::Point(345, 371)); 
  cv::Mat cut_mat(frame, cut_rect);
  //cv::rectangle(frame, cut_rect, cv::Scalar(255, 0, 0));
	#if !(defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__))
	save_debug_img("SmallBox.jpg", frame);
	#endif
	return undefined_bc;
}
box_color_t VisionGetBigBox(const cv::Mat& frame) {
  cv::Rect cut_rect(cv::Point(400, 400), cv::Point(345, 371));
  cv::Mat cut_mat(frame, cut_rect);
  // cv::rectangle(frame, cut_rect, cv::Scalar(255, 0, 0));
#if !(defined(WIN32) || defined(_WIN32) || \
      defined(__WIN32) && !defined(__CYGWIN__))
  save_debug_img("SmallBox.jpg", frame);
#endif
  return undefined_bc;
}
