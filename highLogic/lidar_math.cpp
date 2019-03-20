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
          return RobotPoint(field_sett::max_field_width - coordinate, NAN, M_PI + angle);
        },
        [](double angle, double coordinate) {
          return RobotPoint(NAN, field_sett::max_field_height - coordinate,
                            (3 * M_PI / 2) + angle);
        }
    };

    double gamma = get_gamma_angle(b, c, alpha);
    double angle = M_PI - alpha - gamma;
    double coordinate = c * sin(angle);
    return robot_point_factory[margin](-b_angle_offset - alpha - angle + M_PI_2,
                                       coordinate);
};

RobotPoint get_coordinates_from_corner(double a,
                                                        double b,
                                                        double c,
                                                        double corner_ab,
                                                        double corner_bc,
                                                        double a_angle_offset,
                                                        field_corner corner) {
    RobotPoint
        ans_point = get_coordinates_from_line(a, b, corner_ab,
                                                               a_angle_offset,
                                                               (field_margin) corner);
    ans_point.merge(get_coordinates_from_line(b,
                                                               c,
                                                               corner_bc,
                                                               a_angle_offset
                                                                   + corner_ab,
                                                               field_margin(
                                                                   (corner + 1)
                                                                       % 4)));
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

void get_extr_point_from_obj(const std::vector<Point> &points,
                             size_t begin,
                             size_t end,
                             std::vector<Point> &ans,
                             double delta, std::vector<int> &index) {
    if (end - begin == 1) {
        ans.push_back(points[end]);
        index.push_back(end);
    }
    if (end - begin < 0) {
        return;
    }
    std::pair<size_t, double> max = fiend_max_distant_point(points, points[begin], points[end], begin + 1, end - 1);
    if (max.second <= delta) {
        ans.push_back(points[end]);
        index.push_back(end);
    } else {
        get_extr_point_from_obj(points, begin, max.first, ans, delta, index);
        get_extr_point_from_obj(points, max.first, end, ans, delta, index);
    }
}

//void approx(const std::vector<Point> &points,
//            std::vector<Point> &ans,
//            std::vector<int> &ind) { // нетестировал
//    for (int i = 0; i < ind.size() - 1; i++) {
//        double sum_x = 0;
//        double sum_y = 0;
//        double sum_xx = 0;
//        double sum_xy = 0;
//        auto add = [&](double x, double y) {
//          sum_x += x;
//          sum_y += y;
//          sum_xx += (x * x);
//          sum_xy += (x * y);
//        };
//        int count = 0;
//        for (int k = ind[i] + 1; k < (ind[i + 1] - 1);
//             k = (k + 1) % points.size()) {
//            add(points[k].get_x(), points[k].get_y());
//            count++;
//        }
//        add(ans[i].get_x(), ans[i].get_y());
//        add(ans[i + 1].get_x(), ans[i + 1].get_y());
//        count += 3;
//        double x = sum_x / (count);
//        double y = sum_y / (count);
//        double xx = sum_xx / (count);
//        double xy = sum_xy / (count);
//        double b = (xy - x * y) / (xx - x * x);
//        double a = y - b * x;
//        // расчёт из a*x+b
//    }
//}

void get_corners_from_obj(const std::vector<Point> &points,
                          size_t begin,
                          size_t end,
                          std::vector<Point> &ans,
                          double delta) {
    /*
     *     Есть проблема, если какаето прямая оказалась || прямой,
     * относительно которой мы ищим точку раздела. Тогда эта точка может остаться.
     * Чтобы её убрать нужно запустить полсе выполнения этой функции проверку на
     * наличие "лишних" точек. При этом обход проверки следует начать со 2 с
     * начала точки. Нужно выбрать именно эту, т. к. через 3 точки нельзя провести
     * две || прямые.
     */
    std::vector<int> ind;
    get_extr_point_from_obj(points, begin, end, ans, delta, ind);
    for (int i = 1; i < ans.size() - 1; i++) {
        if (dist_line2point(ans[i - 1], ans[i + 1], ans[i]) <= delta) {
            ans.erase(ans.begin() + i);
        }
    }
}

std::vector<std::vector<Point>> get_corners(const std::vector<PolarPoint> &polar_points,
                                            show_img_debug debug) {
    std::vector<Point> cartesian_points(polar_points.size());
    for (size_t i = 0; i < polar_points.size(); i++) {
        cartesian_points[i] = polar_points[i].to_cartesian(-M_PI, true);
    }
    std::vector<std::vector<Point>> group_points = get_groups_obj(cartesian_points);
    std::vector<std::vector<Point>> ans;
    for (int i = 0; i < group_points.size(); i++) {
        ans.emplace_back();
        ans.back().push_back(group_points[i][0]);
        get_corners_from_obj(group_points[i], 0, group_points[i].size() - 1, ans.back());
    }
    if (debug != nullptr) {
        DebugFieldMat mat;
        add_points_img(mat, cartesian_points);
        add_lines_img(mat, ans);
        debug("get_corners_lidar_math", mat);
    }
    return ans;
}

inline bool is_parallel(double angel1, double angel2, double truncation_error) {
    angel1 = (angel1 < 0) ? (angel1 + M_PI) : (angel1);
    angel2 = (angel2 < 0) ? (angel2 + M_PI) : (angel2);
    return ((fabs(angel2 - angel1) < truncation_error)
            || (((M_PI - std::max(angel1, angel2)) + std::min(angel1, angel2))
                < truncation_error));
}

bool parallel_lines(const Point &line1_a, const Point &line1_b,
                    const Point &line2_a, const Point &line2_b,
                    double truncation_error) {
    double angel1 = atan2(line1_a.get_x() - line1_b.get_x(),
                          line1_a.get_y() - line1_b.get_y());
    double angel2 = atan2(line2_a.get_x() - line2_b.get_x(),
                          line2_a.get_y() - line2_b.get_y());
    return is_parallel(angel1, angel2, truncation_error);
}

std::vector<std::vector<std::pair<Point, line_t>>> line2line_type(
    const std::vector<std::vector<Point>> &points) {
    std::vector<std::vector<std::pair<Point, line_t>>> ans(points.size());
    for (int i = 0; i < points.size(); i++) {
        ans[i].emplace_back();
        ans[i].resize(points[i].size());
        for (int j = 0; j < points[i].size(); j++) {
            ans[i][j].first = points[i][j];
            ans[i][j].second = undefined_lt;
        }
    }
    return ans;
}

void detect_boarder(std::vector<std::vector<std::pair<Point, line_t>>> &points,
                    double error_angel, double delta) {
    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < (points[i].size() - 1); j++) {
            if ((points[i][j].first.dist(points[i][j + 1].first)) > 5
                * (field_sett::climate_box_max
                    + field_sett::truncation_field_error
                    + lidar_sett::truncation_error)) {
                points[i][j].second = border_lt;
            }
            for (int x = i; x < points.size(); x++) {
                for (int y = (x == i) ? (j + 1) : (0); y < (points[x].size() - 1); y++) {
                    if (parallel_lines(points[i][j].first,
                                       points[i][j + 1].first,
                                       points[x][y].first,
                                       points[x][y + 1].first,
                                       error_angel)
                        && (dist_line2point(points[i][j].first,
                                            points[i][j + 1].first,
                                            Point((points[x][y].first.get_x()
                                                      + points[x][y
                                                          + 1].first.get_x())
                                                      / 2,
                                                  (points[x][y].first.get_y()
                                                      + points[x][y
                                                          + 1].first.get_y())
                                                      / 2)) >= (field_sett::min_field - delta))
                                                      ) {
                        points[i][j].second = border_lt;
                        points[x][y].second = border_lt;
                    }
                }
            }
        }
    }
}

bool is_real_size_line_in(std::vector<std::vector<std::pair<Point, line_t>>> &points,
                          int i,
                          int j,
                          double max) { // возвращает True когда мы видим отрезок целиком и он попадает в заданные границы
    if (j >= (points[i].size() - 1)) {
        std::cerr
            << "lidar_math.cpp: is_real_size_line_in: is_real_size_line_in: \n"
            << " Wrong j parameter!"
            << std::endl;
        return false;
    }
    double dist = points[i][j].first.dist(points[i][j + 1].first);
    if (dist <= max) {
        Point zero(0, 0);
        if (j == 0) {
            int last_i = ((i - 1) + points.size()) % points.size();
            double d1 = zero.dist(points[i][j].first);
            double d2 = zero.dist(points[last_i].back().first);
            if (d1 > d2) {
                return false;
            }
        }
        if (j == (points[i].size() - 2)) {
            int next_i = ((i + 1) + points.size()) % points.size();
            double d1 = zero.dist(points[i][j].first);
            double d2 = zero.dist(points[next_i][0].first);
            if (d1 > d2) {
                return false;
            }
        }
        return true;
    }
    return false;
}

void detected_parking_zone(std::vector<std::vector<std::pair<Point, line_t>>> &points,
                           double error_angel) {
    double ang1_board = 0;
    double ang2_board = 0;
    for (int i = 0; i < points.size(); i++) {
        bool break_f = false;
        for (int j = 0; j < (points[i].size() - 1); j++) {
            if (points[i][j].second == border_lt) {
                ang1_board = atan2(points[i][j + 1].first.get_y() - points[i][j].first.get_y(),
                                   points[i][j + 1].first.get_x() - points[i][j].first.get_x());
                ang2_board = ang1_board + ((ang1_board < 0) ? (1) : (-1)) * M_PI_2;
                break_f = true;
                break;
            }
        }
        if (break_f) {
            break;
        }
    }
    double width_error_pz = lidar_sett::max_tr_error
                            + field_sett::truncation_field_error
                            + field_sett::parking_zone_thickness;
    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < (points[i].size() - 1); j++) {
            double dist = points[i][j + 1].first.dist(points[i][j].first);
            if ((points[i][j].second == undefined_lt)
                && (((dist >= (field_sett::parking_zone_width_min
                            - width_error_pz)
                    && is_real_size_line_in(points, i, j, field_sett::parking_zone_width_max
                        + width_error_pz))
                    || ((dist >= field_sett::size_field_unit)
                        && (ang2_board != ang1_board)
                        && !is_parallel(ang1_board,
                            atan2(points[i][j + 1].first.get_y()
                                      - points[i][j].first.get_y(),
                                  points[i][j + 1].first.get_x()
                                      - points[i][j].first.get_x()),
                            error_angel)
                        && !is_parallel(ang2_board,
                            atan2(points[i][j + 1].first.get_y()
                                      - points[i][j].first.get_y(),
                                  points[i][j + 1].first.get_x()
                                      - points[i][j].first.get_x()),
                        error_angel))
                    ) && (dist <= (field_sett::parking_zone_width_max
                    + width_error_pz)))
                    ) {
                for (int k = 0; k < points[i].size(); k++) {
                    points[i][k].second = parking_lt;
                }
            }
        }
    }
};
