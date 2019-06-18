#include "VisionAlgs.h"
#include <opencv2/imgproc.hpp>
#include "debug.h"
#include <algorithm>
color_t GetSquareBin(cv::Mat frame);
struct bin_hsv_params {
  std::pair<int, int> hue;
  std::pair<int, int> sut;
  std::pair<int, int> val;
};

bin_hsv_params briks_colors[8]{
    {{0, 0}, {0, 0}, {0, 0}},            // undefined_c
    {{89, 115}, {76, 192}, {0, 255}},  // blue_c
    {{157, 176}, {71, 154}, {0, 255}},  // red_c
    {{55, 85}, {42, 101}, {0, 255}},  // green_c
    {{1, 11}, {71, 154}, {0, 255}},  // orange_c
    {{16, 38}, {49, 137}, {0, 255}},  // yellow_c
    {{0, 255}, {20, 42}, {0, 111}},  // black_c
    {{0, 255}, {0, 17}, {120, 255}}   // white_c
};

color_t VisionGetSmallBox(const cv::Mat& frame) {
  cv::Rect cut_rect(cv::Point(400, 400), cv::Point(345, 371));
  cv::Mat cut_mat(frame, cut_rect);
 
  color_t  color = GetSquareBin(cut_mat);

  // cv::rectangle(frame, cut_rect, cv::Scalar(255, 0, 0));
#if !(defined(WIN32) || defined(_WIN32) || \
      defined(__WIN32) && !defined(__CYGWIN__))
  save_debug_img("SmallBox.jpg", frame);
#endif
  return color;
}
color_t VisionGetBigBox(const cv::Mat& frame,double dist) {
  const double dist_coef = 0.1043478260869565;

  //far 355    near 505     off 120  385
  cv::Rect cut_rect(cv::Point(555, 505 - (dist * dist_coef)), cv::Size(20, 20));
  cv::Mat cut_mat(frame, cut_rect);
  cv::Mat f_with_rect;
  frame.copyTo(f_with_rect);
  cv::rectangle(f_with_rect, cut_rect, cv::Scalar(255, 5, 44));
  color_t color = GetSquareBin(cut_mat);
#if !(defined(WIN32) || defined(_WIN32) || \
      defined(__WIN32) && !defined(__CYGWIN__))
  save_debug_img("BigBox.jpg", frame);
  save_debug_img("BigBoxRect.jpg", f_with_rect);
#endif
  return color;  // undefined_c;
}

color_t GetSquareBin(cv::Mat frame) {
  const int kMorthSize = 3;
  double area = 0;
  std::vector<int> areas_max;
  color_t color = undefined_c;
  cv::Mat src_hsv, src_hsv_bin, src_canny;
  cv::cvtColor(frame, src_hsv, cv::COLOR_BGR2HSV);
  std::vector < std::vector < cv::Point >> contours, brickContours;
  std::vector<cv::Point> brickContour;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Vec3f> circles;

  for (int i = 0; i < 8; ++i) {
    int area_max = 0;
  inRange(src_hsv,
            cv::Scalar(briks_colors[i].hue.first, briks_colors[i].sut.first,briks_colors[i].val.first),
            cv::Scalar(briks_colors[i].hue.second, briks_colors[i].sut.second,briks_colors[i].val.second),
            src_hsv_bin);

  cv::erode(src_hsv_bin, src_hsv_bin,
            getStructuringElement(cv::MORPH_ELLIPSE,
								      cv::Size(kMorthSize, kMorthSize)));
  cv::dilate(src_hsv_bin, src_hsv_bin,
             getStructuringElement(cv::MORPH_ELLIPSE,
                                   cv::Size(kMorthSize, kMorthSize)));
  cv::dilate(src_hsv_bin, src_hsv_bin,
             getStructuringElement(cv::MORPH_ELLIPSE,
                                   cv::Size(kMorthSize, kMorthSize)));
  cv::erode(src_hsv_bin, src_hsv_bin,
            getStructuringElement(cv::MORPH_ELLIPSE,
                                  cv::Size(kMorthSize, kMorthSize)));
  cv::findContours(src_hsv_bin, contours, hierarchy, cv::RETR_CCOMP,
               cv::CHAIN_APPROX_NONE);
  for (int j = 0; j < contours.size(); ++j) {
    area = cv::contourArea(contours[j]);
    if (area > area_max) {
      brickContour = contours[j];
      area_max = area;
      break;
    }
  }

	areas_max.push_back(area_max);
  }
  int area_max = 0;
  auto it = std::max_element(areas_max.begin(),areas_max.end());
 
  return (color_t)std::distance(areas_max.begin(), it);
}