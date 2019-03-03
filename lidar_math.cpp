//
// Created by Danila on 29.01.2019.
//
// Все углы - в радианах!
// Все растояния - в милиметрах
//
// Дополнительно:
// 1) Преобразование Хафа - https://pastebin.com/LvUFDmVJ
// 2) Поиск линии за линию (не рабочий по идее) - https://pastebin.com/gEBhtAfU
//

#include <cmath>
#include <algorithm>
#include <assert.h>
#include <iostream>
#include <vector>
#include <array>
#include "lidar_math.h"

inline double get_side_triangle(double b, double c, double alpha) {
    return sqrt(b * b + c * c - 2 * c * b * cos(alpha));
}

inline double get_gamma_angle(double b, double c, double alpha) {
    double side = get_side_triangle(b, c, alpha);
    return acos((side * side + b * b - c * c) / (2 * side * b));
};

RobotPoint get_coordinates_from_line(double b, double c, double alpha,
                                     double b_angle_offset,
                                     field_margin margin) {
    // b_angle_offset - всегда указывается между 0 и само левой, если это
    // отрицательный offset, иначе к самой ближней по часовой стрелке
    // Все лини даются в обходе по часовой стрелке

    RobotPoint(*robot_point_factory[4])(double, double) = {
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
                                                     (field_margin) corner);
    ans_point.merge(get_coordinates_from_line(b, c, corner_bc, a_angle_offset
                                                  + corner_ab,
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

std::vector<std::vector<Point>> get_groups_obj(const std::vector<Point> &points,
                                               double max_dist) {
    std::vector<std::vector<Point>> lines;
    lines.emplace_back();
    lines.back().push_back(points[0]);
    for (int i = 1; i < points.size(); i++) {
        if (lines.back().back().dist(points[i]) > max_dist) {
            if ((lines.back().size() < 3) && (lines.size() > 1)) {
                lines.back().clear();
            } else {
                lines.emplace_back();
            }
        }
        lines.back().push_back(points[i]);
    }
    if ((lines.size() > 1) && (lines[0][0].dist(lines.back().back()) <= max_dist)) {
        for (int i = 0; i < lines[0].size(); i++) {
            lines.back().push_back(lines[0][i]);
        }
        lines.erase(lines.begin());
    }
    if (lines.back().size() < 3) {
        lines.erase(lines.end());
    }
    return lines;
}

inline double dist_line2point(const Point &line_a,
                              const Point &line_b,
                              const Point &p) {
    const Point
        delta(line_b.get_x() - line_a.get_x(), line_b.get_y() - line_a.get_y());
    return fabs(delta.get_y() * p.get_x()
                    - delta.get_x() * p.get_y()
                    + line_b.get_x() * line_a.get_y()
                    - line_b.get_y() * line_a.get_x())
        / sqrt(delta.get_x() * delta.get_x() + delta.get_y() * delta.get_y());
}

std::pair<size_t, double> fiend_max_distant_point(const std::vector<Point> &points, const Point &line_a, const Point &line_b, const size_t begin, const size_t end) {
    double max_dist = 0;
    size_t index = begin;
    for (size_t i = begin; i <= end; i++) {
        double dist = dist_line2point(line_a, line_b, points[i]);
        if (dist > max_dist) {
            max_dist = dist;
            index = i;
        }
    }
    return std::pair<size_t, double>{index, max_dist};
}

void get_corners_from_obj(const std::vector<Point> &points,
                          size_t begin,
                          size_t end,
                          std::vector<Point> &ans,
                          double delta) {
    if (end - begin == 1) {
        ans.push_back(points[end]);
    }
    if (end - begin <= 0) {
        return;
    }
    size_t mid = (begin + end) / 2;
    if (dist_line2point(points[begin], points[end], points[mid]) <= delta) {
        std::pair<size_t, double> l_max_ind = fiend_max_distant_point(points, points[begin], points[mid], begin + 1, mid - 1);
        std::pair<size_t, double> r_max_ind = fiend_max_distant_point(points, points[mid], points[end], mid + 1, end - 1);
        if ((l_max_ind.second > delta) || (r_max_ind.second > delta)) {
            if (l_max_ind.second > delta) {
                get_corners_from_obj(points, begin, l_max_ind.first, ans, delta);
                get_corners_from_obj(points, l_max_ind.first, mid, ans, delta);
            }
            if (r_max_ind.second > delta) {
                get_corners_from_obj(points, mid, r_max_ind.first, ans, delta);
                get_corners_from_obj(points, r_max_ind.first, end, ans, delta);
            }
        } else {
            ans.push_back(points[end]);
        }
    } else {
        get_corners_from_obj(points, begin, mid, ans, delta);
        get_corners_from_obj(points, mid + 1, end, ans, delta);
    }
}
