//
// Created by Danila on 29.01.2019.
//

#ifndef LIDAR_MATH_LIDAR_MATH_H
#define LIDAR_MATH_LIDAR_MATH_H

#include <math.h>
#include "logic_structures.h"
#include "settings.h"
#include "debug.h"

RobotPoint init_position_from_line(double b, double c, double alpha);
RobotPoint init_position_from_corner(double a, double b, double c,
                                     double corner_ab, double corner_bc,
                                     double a_angle_offset);
std::vector<std::vector<Point>> get_corners(const std::vector<PolarPoint> &polar_points,
                                            show_img_debug debug = nullptr);
void detect_boarder(std::vector<std::vector<std::pair<Point, line_t>>> &points,
                    double error_angel = field_sett::parking_zone_angel_min / 2,
                    double delta = field_sett::number_field_unit);
void detected_parking_zone(std::vector<std::vector<std::pair<Point, line_t>>> &points,
                           double error_angel = field_sett::parking_zone_angel_min / 2);
std::vector<std::vector<std::pair<Point, line_t>>> line2line_type(
    const std::vector<std::vector<Point>> &points);
void detected_box(std::vector<std::vector<std::pair<Point, line_t>>> &points);
bool is_parallel(double angel1, double angel2,
                 double truncation_error = field_sett::parking_zone_angel_min / 2);
void detected_lines_type(std::vector<std::vector<std::pair<Point, line_t>>> &points);
std::vector<std::vector<std::pair<Point, line_t >>> detected_lines_and_types(
    const std::vector<PolarPoint> &points);

//---FOR TEST---//
RobotPoint get_coordinates_from_line(double b, double c, double alpha,
                                     double b_angle_offset,
                                     field_margin margin);
RobotPoint get_coordinates_from_corner(double a, double b, double c,
                                       double corner_ab, double corner_bc,
                                       double a_angle_offset,
                                       field_corner corner);
std::vector<std::vector<Point>> get_groups_obj(const std::vector<Point> &points,
                                           double max_dist = field_sett::size_field_unit);
void get_corners_from_obj(const std::vector<Point> &points,
                          size_t begin,
                          size_t end,
                          std::vector<Point> &ans,
                          double delta = M_SQRT2 * 2
                              * lidar_sett::truncation_error);
bool is_real_size_line_in(std::vector<std::vector<std::pair<Point, line_t>>> &points,
                          int i,
                          int j,
                          double max);
bool is_convex_triangle(std::vector<std::vector<std::pair<Point, line_t>>> &points,
                        int i,
                        int j);
//////

#endif //LIDAR_MATH_LIDAR_MATH_H
