//
// Created by Danila on 11.03.2019.
//

#include "debug.h"
#include "settings.h"
#include <ctime>
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <iostream>

std::ofstream log_text_out(log_path + log_out_text_file_name);

std::string get_log_name(const std::string &s) {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[100];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%H:%M:%S_%Y_%m_%d", timeinfo);
    return buffer + s;
}

void show_debug_img(const std::string &s, const cv::Mat &img) {
    std::string name = get_log_name(s);
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE );
    imshow(name, img);
    cv::waitKey(0);
}

void save_debug_img(const std::string &s, const cv::Mat &img) {
    cv::imwrite(log_path + get_log_name(s) + ".jpg", img);
}

void clear_logs() {
	std::string command;
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        const char *delete_command = "rmdir /Q /S ";
        const char *create_dir_command = "md ";
    #else
	    const char *archive_command  = "cp -r ";
        const char *delete_command = "rm -r ";
        const char *create_dir_command = "mkdir ";
		command = archive_command + log_path + " " + archive_path  + get_log_name("logs-");
		std::system(command.c_str());
    #endif
    command = delete_command + log_path;
    std::system(command.c_str());
	
    command = create_dir_command + log_path;
    std::system(command.c_str());
	
	log_text_out.open(log_path + log_out_text_file_name);
}

void add_points_img(DebugFieldMat &mat, const std::vector<Point> &points, const cv::Scalar &color) {
    if (mat.zoom == 0) {
        Point min_corner(points[0]), max_corner(points[0]);
        for (size_t i = 1; i < points.size(); i++) {
            min_corner.set_x(std::min(min_corner.get_x(), points[i].get_x()));
            min_corner.set_y(std::min(min_corner.get_y(), points[i].get_y()));
            max_corner.set_x(std::max(max_corner.get_x(), points[i].get_x()));
            max_corner.set_y(std::max(max_corner.get_y(), points[i].get_y()));
        }
        mat.set_param(min_corner, max_corner);
    }
    for (auto i : points) {
        Point p = mat.get_zoom_point(i);
        if ((p.get_x() > 0) && (p.get_y() > 0) && (p.get_x() < mat.size().width)
            && (p.get_x() < mat.size().height))
            cv::circle(mat, {int(std::round(p.get_x())), int(std::round(p.get_y()))}, 1, color, cv::FILLED);
    }
}

void add_lines_img(DebugFieldMat &mat, const std::vector<std::vector<Point>> &points, const cv::Scalar &color, const cv::Scalar &color_corn) {
    if (mat.zoom == 0) {
        Point min_corner(points[0][0]), max_corner(points[0][0]);
        for (size_t i = 0; i < points.size(); i++) {
            for (size_t j = 0; j < points[i].size(); j++) {
                min_corner.set_x(std::min(min_corner.get_x(), points[i][j].get_x()));
                min_corner.set_y(std::min(min_corner.get_y(), points[i][j].get_y()));
                max_corner.set_x(std::max(max_corner.get_x(), points[i][j].get_x()));
                max_corner.set_y(std::max(max_corner.get_y(), points[i][j].get_y()));
            }
        }
        mat.set_param(min_corner, max_corner);
    }
    for (size_t j = 0; j < points.size(); j++) {
        cv::Scalar cl_line = color;
        cl_line[0] = 255.0 / points.size() * j;
        for (size_t i = 1; i < points[j].size(); i++) {
            Point a = mat.get_zoom_point(points[j][i - 1]);
            Point b = mat.get_zoom_point(points[j][i]);
            cv::line(mat, {int(std::round(a.get_x())), int(std::round(a.get_y()))},
                     {int(std::round(b.get_x())), int(std::round(b.get_y()))}, cl_line, 2);
            cv::circle(mat,
                       {int(std::round(a.get_x())), int(std::round(a.get_y()))},
                       2,
                       color_corn,
		        cv::FILLED);
        }
        Point b = mat.get_zoom_point(points[j].back());
	    cv::circle(mat, { int(std::round(b.get_x())), int(std::round(b.get_y())) }, 2, color_corn, cv::FILLED);
    }
}

void add_lines_img(DebugFieldMat &mat,
                   const std::vector<std::vector<std::pair<Point,line_t>>> &points,
                   bool writing, const cv::Scalar &color_corn) {
    const cv::Scalar colors[] = {{240, 33, 23},
                                 {251, 238, 9},
                                 {39, 159, 211},
                                 {206, 164, 223}};
    if (mat.zoom == 0) {
        Point min_corner(points[0][0].first), max_corner(points[0][0].first);
        for (size_t i = 0; i < points.size(); i++) {
            for (size_t j = 0; j < points[i].size(); j++) {
                min_corner.set_x(std::min(min_corner.get_x(),
                                          points[i][j].first.get_x()));
                min_corner.set_y(std::min(min_corner.get_y(),
                                          points[i][j].first.get_y()));
                max_corner.set_x(std::max(max_corner.get_x(),
                                          points[i][j].first.get_x()));
                max_corner.set_y(std::max(max_corner.get_y(),
                                          points[i][j].first.get_y()));
            }
        }
        mat.set_param(min_corner, max_corner);
    }
    for (size_t j = 0; j < points.size(); j++) {
        for (size_t i = 1; i < points[j].size(); i++) {
            Point a = mat.get_zoom_point(points[j][i - 1].first);
            Point b = mat.get_zoom_point(points[j][i].first);
            cv::line(mat,
                     {int(std::round(a.get_x())), int(std::round(a.get_y()))},
                     {int(std::round(b.get_x())), int(std::round(b.get_y()))},
                     colors[points[j][i - 1].second],
                     2);
            cv::circle(mat,
                       {int(std::round(a.get_x())), int(std::round(a.get_y()))},
                       2,
                       color_corn,
		        cv::FILLED);
            if (writing) {
                cv::putText(mat,
                            "{" + std::to_string(int(std::round(points[j][i - 1].first.get_x()))) + ", "
                                + std::to_string(int(std::round(points[j][i - 1].first.get_y()))) + "}",
                            {int(std::round(a.get_x())),
                             int(std::round(a.get_y()))},
		            cv::FONT_HERSHEY_SIMPLEX, 0.3,
                            {255, 255, 255});
            }
        }
        Point b = mat.get_zoom_point(points[j].back().first);
        cv::circle(mat,
                   {int(std::round(b.get_x())), int(std::round(b.get_y()))},
                   2,
                   color_corn,
		    cv::FILLED);
        if (writing) {
            cv::putText(mat,
                        "{" + std::to_string(int(std::round(points[j].back().first.get_x()))) + ", "
                            + std::to_string(int(std::round(points[j].back().first.get_y()))) + "}",
                        {int(std::round(b.get_x())),
                         int(std::round(b.get_y()))},
		        cv::FONT_HERSHEY_SIMPLEX, 0.3,
                        {255, 255, 255});
        }
    }
}

void add_point_img(DebugFieldMat &mat, const Point &p, const cv::Scalar &circle_color) {
	cv::circle(mat, { int(mat.get_zoom_point(p).get_x()), int(mat.get_zoom_point(p).get_y()) }, 3, { 0, 0, 255 }, cv::FILLED);
}

void add_robot_img_global(DebugFieldMat &mat,
                          const RobotPoint r_p,
                          const cv::Scalar &circle_color) {
    RobotPoint p(r_p.get_x(), r_p.get_y(), r_p.get_angle());
    const double radius_line = 12;
	                          cv::circle(mat, { int(p.get_x() * mat.zoom + mat.indent), int(p.get_y()*mat.zoom + mat.indent) }, 7, circle_color, cv::FILLED);
    cv::line(mat, {int(p.get_x() * mat.zoom + mat.indent), int(p.get_y()*mat.zoom + mat.indent)},
             {int((p.get_x() * mat.zoom + mat.indent) + radius_line * cos(M_PI - p.get_angle()) ),
              int((p.get_y()*mat.zoom + mat.indent) - radius_line * sin(M_PI - p.get_angle()))}, {0, 0, 255}, 2);
}

void save_ld_data(const std::vector<PolarPoint> &p, const std::string &s) {
	std::ofstream f;
	f.open(log_path + get_log_name(s) + ".ld");
	bool d = f.is_open();
	for (auto i : p)
	{
		f << i.get_r() << " " << i.get_f() << std::endl;
	}
	f.close();
}

void write_log(const std::string &s) {
    log_text_out << get_log_name(": ") << s << std::endl;
}
void cout_to_file_log_enable()
{
	std::ofstream out(log_path + "cout.log");
	auto cinbuf = std::cout.rdbuf(out.rdbuf());
}