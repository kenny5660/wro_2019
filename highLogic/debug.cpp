//
// Created by Danila on 11.03.2019.
//

#include "debug.h"
#include <ctime>
#include <chrono>
#include <opencv2/imgproc.hpp>

std::string get_log_name(const std::string &s) {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[100];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S_", timeinfo);
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
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        const char *delete_command = "rmdir /Q /S ";
        const char *create_dir_command = "md ";
    #else
        const char *delete_command = "rm -r ";
        const char *create_dir_command = "mkdir ";
    #endif
    std::string command = delete_command + log_path;
    std::system(command.c_str());
    command = create_dir_command + log_path;
    std::system(command.c_str());
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
            cv::circle(mat, {int(std::round(p.get_x())), int(std::round(p.get_y()))}, 1, color, CV_FILLED);
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
            cv::circle(mat, {int(std::round(a.get_x())), int(std::round(a.get_y()))}, 2, color_corn, CV_FILLED);
        }
        Point b = mat.get_zoom_point(points[j].back());
        cv::circle(mat, {int(std::round(b.get_x())), int(std::round(b.get_y()))}, 2, color_corn, CV_FILLED);
    }
}
