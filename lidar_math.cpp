//
// Created by Danila on 29.01.2019.
//
// Все углы - в радианах!
// Все растояния - в милиметрах
//

#include <cmath>
#include <assert.h>
#include <iostream>
#include <vector>
#include "lidar_math.h"

template <class T>
inline int sgn(T a) {
    return (a < 0) ? (-1) : 1;
}

inline double get_side_triangle(double b, double c, double alpha) {
    double co = cos(alpha);
    double bb = 2 * c * b * co;
    return sqrt(b * b + c * c - bb);
}

inline double get_gamma_angle(double b, double c, double alpha) {
    double side = get_side_triangle(b, c, alpha);
    double a = asin((sin(alpha) * c) / side);
    return (c > b) ? (M_PI - a) : a;
};

RobotPoint get_coordinate(double b, double l, double c, double alpha) {
    double angle = M_PI - alpha / 2 - get_gamma_angle(b, c, alpha);
    double x = l * sin(angle);
    return RobotPoint(x, 0, angle);
};

RobotPoint get_coordinates_from_line(double b, double c, double alpha,
                                     double b_angle_offset,
                                     field_margin margin) {
    // b_angle_offset - всегда указывается между 0 и само левой, если это
    // отрицательный offset, иначе к самой ближней по часовой стрелке

    RobotPoint (*robot_point_factory[4])(double, double) = {
        [](double angle, double coordinate) {
          return RobotPoint(coordinate, NAN, angle);
        },
        [](double angle, double coordinate) {
          return RobotPoint(NAN, coordinate, M_PI_2 + angle);
        },
        [](double angle, double coordinate) {
          return RobotPoint(MAX_FIELD_WIDTH - coordinate, NAN, M_PI + angle);
        },
        [](double angle, double coordinate) {
          return RobotPoint(NAN, MAX_FIELD_HEIGHT - coordinate,
                            (3 * M_PI / 2) + angle);
        }
    };

    double gamma = get_gamma_angle(b, c, alpha);
    double angle = M_PI - alpha - gamma;
    double coordinate = c * sin(angle);
    return robot_point_factory[margin](-b_angle_offset - alpha - angle + M_PI_2,
                                       coordinate);
};

RobotPoint get_coordinates_from_corner(double a, double b, double c,
                                       double corner_ab, double corner_bc,
                                       double a_angle_offset,
                                       field_corner corner) {
    RobotPoint ans_point = get_coordinates_from_line(a, b, corner_ab,
                                                     a_angle_offset,
                                                     (field_margin)corner);
    ans_point.merge(get_coordinates_from_line(b, c, corner_bc, a_angle_offset
                                              + sgn(a_angle_offset) * corner_ab,
                                              field_margin((corner + 1) % 4)));
    return ans_point;
}

RobotPoint init_position_from_line(double b, double c, double alpha) {
    return get_coordinates_from_line(b, c, alpha, -alpha / 2,
                                     left_field_margin);
}

RobotPoint init_position_from_corner(double a, double b, double c,
                                     double corner_ab, double corner_bc,
                                     double a_angle_offset) {
    return get_coordinates_from_corner(a, b, c, corner_ab, corner_bc,
                                       a_angle_offset, top_left_corner);
}

std::vector<Point> polar2cartesian(const std::vector<Point> &polar_points,
                                   const Point &offset) {

}

std::vector<Point> get_corners(std::vector<Point> points) {

}
