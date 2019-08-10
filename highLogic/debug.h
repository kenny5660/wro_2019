//
// Created by Danila on 11.03.2019.
//

#ifndef LIDAR_MATH_DEBUG_H
#define LIDAR_MATH_DEBUG_H

#include <string>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "logic_structures.h"

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	const std::string log_path = "..\\log\\";
#else
	const std::string log_path = "./log/";
  const std::string archive_path = "./archive/"; 
#endif

const std::string log_out_text_file_name = "log.txt";


const size_t debug_width_img = 500;
const size_t debug_height_img = debug_width_img;

typedef void(*show_img_debug)(const std::string &s, const cv::Mat &img);
void show_debug_img(const std::string &s, const cv::Mat &img);
void save_debug_img(const std::string &s, const cv::Mat &img);

void clear_logs();

void write_log(const std::string &s);

class DebugFieldMat : public cv::Mat {
public:
	DebugFieldMat()
		: Mat(cv::Size(debug_width_img, debug_height_img), CV_8UC3) {
	}

    void add_points(const std::vector<Point> &points,
                    const cv::Scalar &color = {7, 178, 55});
    void add_lines(const std::vector<std::vector<Point>> &points,
                   const cv::Scalar &color = { 128, 123, 190 },
                   const cv::Scalar &color_corn = { 255, 255, 255 });
    void add_lines(const std::vector<std::vector<std::pair<Point,
                                                           line_t>>> &points,
                   bool writing = false,
                   const cv::Scalar &color_corn = { 128, 123, 190 });
    void add_point(const Point &p = {0, 0},
                   const cv::Scalar &circle_color = {0, 0, 255});
    void add_robot_global(const RobotPoint r_p,
                          const cv::Scalar &circle_color = {106, 103, 107});

    void show(const std::string s = "");

 public:
	Point get_zoom_point(const Point &p) const {
		return {
			 (p.get_x() - offset.get_x()) * zoom + indent,
			debug_height_img - (p.get_y() - offset.get_y()) * zoom - indent
		 };
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
	Point offset = { 0, 0 };

	const size_t indent = 10;
};

void save_ld_data(const std::vector<PolarPoint> &p, const std::string &s = "");

void cout_to_file_log_enable();
#endif //LIDAR_MATH_DEBUG_H
