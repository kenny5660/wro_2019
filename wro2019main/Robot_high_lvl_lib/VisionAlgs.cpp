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
    {{237, 255}, {140, 226}, {0, 255}},  // blue_c
    {{237, 255}, {140, 226}, {0, 255}},  // red_c
    {{237, 255}, {140, 226}, {0, 255}},  // green_c
    {{237, 255}, {140, 226}, {0, 255}},  // orange_c
    {{237, 255}, {140, 226}, {0, 255}},  // yellow_c
    {{237, 255}, {140, 226}, {0, 255}},  // black_c
    {{237, 255}, {140, 226}, {0, 255}}   // white_c
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
color_t VisionGetBigBox(const cv::Mat& frame) {
  cv::Rect cut_rect(cv::Point(400, 400), cv::Point(345, 371));
  cv::Mat cut_mat(frame, cut_rect);

  // cv::rectangle(frame, cut_rect, cv::Scalar(255, 0, 0));
#if !(defined(WIN32) || defined(_WIN32) || \
      defined(__WIN32) && !defined(__CYGWIN__))
  save_debug_img("SmallBox.jpg", frame);
#endif
  return undefined_c;
}

color_t GetSquareBin(cv::Mat frame) {
  const int kMorthSize = 3;
  const int AREA_min = 20;
  const int AREA_max = 1000;
  double area = 0;
  std::vector<int> areas_max;
  color_t color = undefined_c;
  cv::Mat src_hsv, src_hsv_bin, src_canny;
  cvtColor(frame, src_hsv, cv::COLOR_BGR2HSV);
  std::vector<std::vector<cv::Point> > contours, brickContours;
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
  cv::Canny(src_hsv_bin, src_canny, 200, 50, 3);
  cv::findContours(src_canny, contours, hierarchy, cv::RETR_CCOMP,
               cv::CHAIN_APPROX_NONE);
  for (int j = 0; j < contours.size(); ++j) {
    area = cv::contourArea(contours[j]);
    if (area > AREA_min && area < AREA_max && area > area_max) {
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