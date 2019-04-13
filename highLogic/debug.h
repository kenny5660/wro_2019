//
// Created by Danila on 11.03.2019.
//

#ifndef LIDAR_MATH_DEBUG_H
#define LIDAR_MATH_DEBUG_H

#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "logic_structures.h"

const std::string log_path = "..\\log\\";

const size_t debug_width_img = 500;
const size_t debug_height_img = debug_width_img;

typedef void (*show_img_debug)(const std::string &s, const cv::Mat &img);
void show_debug_img(const std::string &s, const cv::Mat &img);
void save_debug_img(const std::string &s, const cv::Mat &img);

void clear_logs();

 class DebugFieldMat : public cv::Mat {
  public:
     DebugFieldMat() : Mat(cv::Size(debug_width_img, debug_height_img), CV_8UC3) {
     }

     Point get_zoom_point(const Point &p) const {
      return {(p.get_x() - offset.get_x()) * zoom + indent,
              debug_height_img - (p.get_y() - offset.get_y()) * zoom - indent};
     }

     void set_param(const Point &min, const Point &max) {
         double delta = std::max(std::max(indent +5.0,
                                 fabs(max.get_x() - min.get_x())),
                                 fabs(max.get_y() - min.get_y()));
         zoom = (std::max(debug_width_img, debug_height_img) - 2 * indent) / delta;
         offset.set_x(min.get_x());
         offset.set_y(min.get_y());
     }

     double zoom = 0;
     Point offset = {0, 0};

  private:
     const size_t indent = 10;
 };

void add_points_img(DebugFieldMat &mat, const std::vector<Point> &points,
                    const cv::Scalar &color = {7, 178, 55});

void add_lines_img(DebugFieldMat &mat,
                   const std::vector<std::vector<Point>> &points,
                   const cv::Scalar &color = {128, 123, 190},
                   const cv::Scalar &color_corn = {255, 255, 255});

void add_lines_img(DebugFieldMat &mat,
                   const std::vector<std::vector<std::pair<Point,line_t>>> &points,
                   const cv::Scalar &color_corn = {128, 123, 190});

#endif //LIDAR_MATH_DEBUG_H