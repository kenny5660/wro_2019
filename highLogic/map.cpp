//
// Created by Danila on 28.03.2019.
//

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <set>
#include "map.h"
#include "settings.h"
#include "lidar_math.h"

// Черновики:
// 1) Добавления мёртвых зон в лист: https://pastebin.com/yX17ZEa2
// 2) Добавление дохлой точки: https://pastebin.com/nbex3pP7
// 3) Определение типа угла: https://pastebin.com/YK6VSfYx
//

enum box_corner_type {
    left_up_bt,
    right_up_bt,
    right_down_bt,
    left_down_bt,
    undef_bt
};

BoxMap::BoxMap(const Point &p, box_color_t color) : color_(color) {
    add_point(p);
}

void BoxMap::add_point(const Point& p) {
    left_up_corner_.merge(MassPoint(p));
}

void BoxMap::set_color(const box_color_t c) {
    if (color_ != undefined_bc) {
        std::cerr << "map::BoxMap::set_color\n Re-announcement color of box!";
        return;
    }
    color_ = c;
}

void BoxMap::merge(const BoxMap &b) {
    left_up_corner_.merge(b.left_up_corner_);
    if (color_ == undefined_bc) {
        color_ = b.color_;
    } else {
        std::cerr << "map::BoxMap::merge\n Re-announcement color of box!";
    }
}

Point BoxMap::get_box_indent() {
    Point box_center = (Point)left_up_corner_ + Point{field_sett::climate_box_width / 2.,
                                               field_sett::climate_box_height / 2.};
    if ((box_center.get_x() < box_center.get_y()) && (box_center.get_x() < (field_sett::max_field_width - box_center.get_x())) && (box_center.get_x() < (field_sett::max_field_height - box_center.get_y()))) {
        return box_center + Point{field_sett::climate_box_width / 2. + field_sett::climate_box_offset / 2., 0};
    } else if ((box_center.get_y() < box_center.get_x()) && (box_center.get_y() < (field_sett::max_field_width - box_center.get_x())) && (box_center.get_y() < (field_sett::max_field_height - box_center.get_y()))) {
        return box_center + Point{0, field_sett::climate_box_height / 2. + field_sett::climate_box_offset / 2.};
    } else if (((field_sett::max_field_width - box_center.get_x()) < box_center.get_x()) && ((field_sett::max_field_width - box_center.get_x()) < box_center.get_y()) && ((field_sett::max_field_width - box_center.get_x()) < (field_sett::max_field_height - box_center.get_y()))) {
        return box_center + Point{-field_sett::climate_box_width / 2. - field_sett::climate_box_offset / 2., 0};
    } else {
        return box_center + Point{0, -field_sett::climate_box_height / 2. - field_sett::climate_box_offset / 2.};
    }
}

Point BoxMap::get_box_corner(unsigned int i, const Point &offset) const {
    i = i % 4;
    const Point point_offset[4] = {{0, 0},
                                   {1, 0},
                                   {1, 1},
                                   {0, 1}};
    const Point offset_offset[4] = {{1, 1},
                                    {-1, 1},
                                    {-1, -1},
                                    {1, -1}};
    const Point box(field_sett::climate_box_width, field_sett::climate_box_height);
    return left_up_corner_ + (point_offset[i] * box) + (offset_offset[i] * offset);
}

Point BoxMap::cross_box_line(const Point &a, const Point &b, const Point &offset) {
    // a - точка начала
    // b - точка конца
    Point min_p {2 * field_sett::max_field_width, 2 * field_sett::max_field_height};
    for (int i = 0; i < 4; i++) {
        Point p_a = get_box_corner(i, offset);
        Point p_b = get_box_corner((i + 1) % 4, offset);
        Point buff = get_line_cross(p_a, p_b, a, b);
        if ((!std::isnan(buff.get_x())) && (a.dist(min_p) > a.dist(buff))) {
            min_p = buff;
        }
    }
    if (2 * field_sett::max_field_width == min_p.get_x()) {
        return {};
    }
    return min_p;
}

std::array<Point, 4> BoxMap::get_corners() const {
    return std::array<Point, 4>{left_up_corner_, left_up_corner_
        + Point(field_sett::climate_box_width, 0),
                                left_up_corner_
                                    + Point(field_sett::climate_box_width,
                                            field_sett::climate_box_height),
                                left_up_corner_
                                    + Point(0, field_sett::climate_box_height)};
}

void Map::add_box2boarder(std::vector<Point> &border, std::vector<Point> &border_from, const Point &offset) {
//    for (auto i : border_from) {
//        border.push_back(i);
//    }
//    int ind_min_p = 0;
//    for (int i = 1; i < border.size(); i++) {
//        if ((border[ind_min_p].get_y() > border[i].get_y())
//            || ((border[ind_min_p].get_y() == border[i].get_y())
//                && ((border[ind_min_p].get_x() < border[i].get_x())))) {
//            ind_min_p = i;
//        }
//    }
//    std::vector<Point> ans;
//    ans.push_back(border[ind_min_p]);
//    Point last_p(- 2 * field_sett::max_field_width, border[ind_min_p].get_y());
//    int k = -1;
//    while (k != ind_min_p) {
//        Point min;
//        double min_ang = 2 * M_PI;
//        for (int i = 0; i < border.size(); i++) {
//            double a = last_p.dist(border[i]);
//            double b = ans.back().dist(border[i]);
//            double c = last_p.dist(ans.back());
//            double ang = M_PI - std::acos(b * b + c * c - a * a) / (2 * b * c);
//            double d = ((border[i].get_x() - last_p.get_x()) * (ans.back().get_y() - last_p.get_y())
//                - (border[i].get_y() - last_p.get_y()) * (ans.back().get_x() - last_p.get_x())); // D = (х3 - х1) * (у2 - у1) - (у3 - у1) * (х2 - х1)
//            if (d == 0) {
//                if (b > a) {
//                    ang = 3 / 2 * M_PI;
//                } else {
//                    ang = M_PI / 2;
//                }
//            } else if (d < 0) {
//                ang += M_PI_2;
//            }
//            ang = fmod(2 * M_PI + ang, 2 * M_PI);
//            if (ang < min_ang) {
//                min_ang = ang;
//                min = border[i];
//                k = i;
//            }
//        }
//        ans.push_back(min);
//    }
//    ans.pop_back();
    // Версия 2
    int ind_start_box_point = -1;
    for (int i = 0; i < border_from.size(); i++) {
        if (!in_outline(border, border_from[i])) {
            ind_start_box_point = i;
            break;
        }
    }
    if (ind_start_box_point == -1) {
        return;
    }
    std::pair<Point, std::pair<int, int>> up_cross; // первое - индекс добавляемого, второе - зоны
    std::pair<Point, std::pair<int, int>> down_cross; // первое - индекс добавляемого, второе - зоны
    bool is_was = false;
    for (int k = ind_start_box_point; (k != ind_start_box_point) || (!is_was); k = (k + 1) % border_from.size()) {
        is_was = true;
        auto buff = get_cross_line_with_outline(border, border_from[k], border_from[((k + 1) % border_from.size())]);
        if (buff.second != -1) {
            up_cross.first = buff.first;
            up_cross.second = std::make_pair(k, buff.second);
            break;
        }
    }
    is_was = false;
    for (int k = ind_start_box_point; (k != ind_start_box_point) || (!is_was); k = ((k + border_from.size() - 1) % border_from.size())) {
        is_was = true;
        auto buff = get_cross_line_with_outline(border, border_from[(k + border_from.size() - 1) % border_from.size()], border_from[k]);
        if (buff.second != -1) {
            down_cross.first = buff.first;
            down_cross.second = std::make_pair(((k + border_from.size() - 1)), buff.second);
            break;
        }
    }
    // Не Костыль, А КОСТЫЛИЩЕ. Специально для абонентов DNDV
    bool was_s = false;
    bool is_include = false;
    for (int i = (up_cross.second.second + 1) % border.size(); (!was_s) || (i != ((down_cross.second.second + 1) % border.size())); i = ((i + 1) % border.size())) {
        was_s = true;
        is_include = is_include || (i == ind_start_box_point);
    }
    if (!is_include) {
        auto buff = up_cross;
        up_cross = down_cross;
        down_cross = buff;
    }
    std::vector<Point> ans;
    if ((up_cross.first.get_x() != border[up_cross.second.second].get_x()) || (up_cross.first.get_y() != border[up_cross.second.second].get_y())) ans.push_back(up_cross.first);
    was_s = false;
    for (int i = (up_cross.second.second + 1) % border.size(); (!was_s) || (i != ((down_cross.second.second + 1) % border.size())); i = ((i + 1) % border.size())) {
        ans.push_back(border[i]);
        was_s = true;
    }
    was_s = false;
    if ((down_cross.first.get_x() != border[down_cross.second.second].get_x()) || (down_cross.first.get_y() != border[down_cross.second.second].get_y())) ans.push_back(down_cross.first);
    for (int i = (down_cross.second.first + 1) % border_from.size(); (!was_s) || (i != ((up_cross.second.first + 1) % border_from.size())); i = ((i + 1) % border_from.size())) {
        ans.push_back(border_from[i]);
        was_s = true;
    }
    border = ans;
    return;

}

void convex_merge(std::vector<Point> &border, std::vector<Point> &border_from, const Point &offset) {
    std::vector<Point> ans;
    for (int i = 0; i < border_from.size(); i++) {
        border.push_back(border_from[i]);
    }
    ans.push_back(border[0]);
    int start_point_ind = 0;
    for (int i = 1; i < border.size(); i++) {
        if ((ans[0].get_y() > border[i].get_y())
            || ((ans[0].get_y() == border[i].get_y())
                && (ans[0].get_x() > border[i].get_x()))) {
            ans[0] = border[i];
            start_point_ind = i;
        }
    }
    Point last_p = {2 * field_sett::max_field_width, ans[0].get_y()};
    int max_point_ind = -1;
    do {
        double min_cos_ang = 1;
        for (int i = 0; i < border.size(); i++) {
            double a = last_p.dist(border[i]);
            double b = last_p.dist(ans.back());
            double c = ans.back().dist(border[i]);
            if (b == a) {
                continue;
            }
            double cos_ang = (b * b + c * c - a * a) / (2 * b * c);
            if (cos_ang < min_cos_ang) {
                min_cos_ang = cos_ang;
                max_point_ind = i;
            }
        }
        last_p = ans.back();
        ans.push_back(border[max_point_ind]);
    } while (start_point_ind != max_point_ind);
    border = ans;
}

bool Map::add_box(const Point &p) {
    for (int i = 0; i < box_count_; i++) {
        if (boxes_[i].get_left_corner_point().dist(p)
            < (field_sett::climate_box_max - field_sett::truncation_field_error
                - lidar_sett::truncation_error)) {
            boxes_[i].add_left_corner_point(p);
            return false;
        }
    }
    if (box_count_ >= boxes_.size()) {
        std::cerr << "Map::add_box:: Corner array overflow!" << std::endl;
        return false;
    }

    boxes_[box_count_].set_left_corner_point(MassPoint(p));

    // Проверка пересечения контуров
    const Point move_offset = {-robot_sett::move_offset, -robot_sett::move_offset};
    borders.emplace_back();
    for (int i = 0; i < 4; i++) {
        borders.back().push_back(boxes_[box_count_].get_box_corner(i, move_offset));
    }
    for (int i = 0; i < (borders.size() - 1); i++) {
        bool is_cross = false;
        for (int p = 0; p < borders.back().size(); p++) {
            for (int q = 0; q < borders[i].size(); q++) {
                if (!std::isnan(get_line_cross(borders.back()[p],  borders.back()[(p + 1) %  borders.back().size()], borders[i][q], borders[i][(q + 1) % borders[i].size()]).get_x())) {
                    is_cross = true;
                    break;
                }
            }
            if (is_cross) {
                break;
            }
        }
        if(is_cross) {
            add_box2boarder(borders.back(), borders[i], move_offset);
            borders.erase(borders.begin() + i);
            i--;
        }
    }
    //

    box_count_++;

    auto unit = get_field_unit(p);
    death_zone_[unit.first][unit.second] = false;
    death_zone_[unit.first][unit.second + 1] = false;
    death_zone_[unit.first + 1][unit.second + 1] = false;
    death_zone_[unit.first + 1][unit.second] = false;
    return true;
}

inline int line_length2box_number(double length) {
    return std::max(1, fdiv(length, field_sett::climate_box_max - field_sett::truncation_field_error - lidar_sett::truncation_error));
}

void Map::add_boxes_in_robot_pos(const Point &corner, const Point &p) {
    Point side(0, 0);
    int boxes_n = line_length2box_number(corner.dist(p));
    Point delta(p.get_x() - corner.get_x(), p.get_y() - corner.get_y());
    if (fabs(delta.get_x()) < fabs(delta.get_y())) {
        side.set_y(-sign(delta.get_y()));
    } else {
        side.set_x(sign(delta.get_x()));
    }
    Point c(corner.get_x(), -corner.get_y());
    for (int i = 0; i < boxes_n; i++) {
        add_box(position_ + (c + (side * i * field_sett::climate_box_max)));
    }
}

box_corner_type get_box_corner_type(const std::vector<std::pair<Point, line_t>> &points, int j) {
    if ((points[j].first.get_x() < 0) && (points[j].first.get_y() < 0)) {
        if (j == 0) {
            return right_down_bt;
        } else if (j == (points.size() - 1)) {
            return left_up_bt;
        } else if ((points.size() >= 3) &&
            (position_relative_line(points[j + 1].first, points[j - 1].first, points[j].first) == 1)){
            return right_up_bt;
        }
    } else if ((points[j].first.get_x() < 0) && (points[j].first.get_y() > 0)) {
        if (j == 0) {
            return left_down_bt;
        } else if (j == (points.size() - 1)){
            return right_up_bt;
        } else if ((points.size() >= 3) &&
            (position_relative_line(points[j + 1].first, points[j - 1].first, points[j].first) == -1)){
            return right_down_bt;
        }
    } else if ((points[j].first.get_x() > 0) && (points[j].first.get_y() > 0)) {
        if (j == 0) {
            return left_up_bt;
        } else if (j == (points.size() - 1)) {
            return right_down_bt;
        } else if ((points.size() >= 3) &&
            (position_relative_line(points[j + 1].first, points[j - 1].first, points[j].first) == -1)){
            return left_down_bt;
        }
    } else {
        if (j == 0) {
            return right_up_bt;
        } else if (j == (points.size() - 1)) {
            return left_down_bt;
        } else if ((points.size() >= 3) &&
            (position_relative_line(points[j + 1].first, points[j - 1].first, points[j].first) == 1)){
            return left_up_bt;
        }
    }
    return undef_bt;
}

void Map::set_death_outline(const std::vector<Point> &outline, bool val) {
    Point max(0, 0);
    Point min(field_sett::max_field_width, field_sett::max_field_height);
    for (auto i : outline) {
        if (max.get_x() < i.get_x()) {
            max.set_x(i.get_x());
        }
        if (max.get_y() < i.get_y()) {
            max.set_y(i.get_y());
        }
        if (min.get_x() > i.get_x()) {
            min.set_x(i.get_x());
        }
        if (min.get_y() > i.get_y()) {
            min.set_y(i.get_y());
        }
    }
    std::pair<int, int> min_unit = get_field_unit(min);
    std::pair<int, int> max_unit = get_field_unit(max);
    for (int i = min_unit.first; i <= max_unit.first; i++) {
        for (int j = min_unit.second; j <= max_unit.second; j++) {
            if (val) {
                death_zone_[i][j] = death_zone_[i][j]
                    || in_outline(outline,
                                  {i * field_sett::size_field_unit
                                       + field_sett::size_field_unit / 2.,
                                   j * field_sett::size_field_unit
                                       + field_sett::size_field_unit / 2.});
            } else {
                death_zone_[i][j] = death_zone_[i][j] &&
                    !in_outline(outline,
                                                {i * field_sett::size_field_unit
                                                     + field_sett::size_field_unit / 2.,
                                                 j * field_sett::size_field_unit
                                                     + field_sett::size_field_unit / 2.});
            }
//            std::cout << i << " " << j << " " << death_zone_[i][j] << std::endl;
//            std::cout << field_sett::size_field_unit
//                + field_sett::size_field_unit / 2. << " " <<
//               j * field_sett::size_field_unit
//                   + field_sett::size_field_unit / 2. << std::endl;
        }
    }
}

void Map::delete_from_death_zone_circle(const Point &p, double r) {
    int n = fdiv(r, field_sett::size_field_unit) + 2;
    auto p_unit = get_field_unit(p);
    auto min = std::make_pair(std::max(0, p_unit.first - n),
                              std::max(0, p_unit.second - n));
    auto max = std::make_pair(std::min(field_sett::number_field_unit - 1, p_unit.first + n),
                              std::min(field_sett::number_field_unit - 1, p_unit.second + n));
    for (int i = min.first; i <= max.first; i++) {
        for (int j = min.second; j <= max.second; j++) {
            Point up(i * field_sett::size_field_unit, j * field_sett::size_field_unit);
            if (up.get_y() < p.get_y()) {
                up.set_y(up.get_y() + field_sett::size_field_unit);
            }
            if (up.get_x() < p.get_x()) {
                up.set_x(up.get_x() + field_sett::size_field_unit);
            }
            //std::cout << i << " " << j << " " << p.dist(up) << " " << (p.dist(up) < r) << std::endl;
            death_zone_[i][j] = death_zone_[i][j] && !(p.dist(up) < r);
        }
    }
}

void Map::add_pz(const Point &p_1, const Point &p_2) {
    for (int i = 0; i < death_zone_.size(); i++) {
        std::fill(death_zone_[i].begin(), death_zone_[i].end(), true);
    }
    Point p1 = normal_point(p_1);
    Point p2(p_2 - (p_1 - p1));
    const int kPoint_offset = 12;
    Point (* const point_offset[kPoint_offset])(double s1, double s2) = { // последний элемент является фальшивым
        [](double s1, double s2){ return Point{sin(atan(-3)) * s1, cos(atan(-3)) * s2}; },
        [](double s1, double s2){ return Point{sin(atan(-2)) * s1, cos(atan(-2)) * s2}; },
        [](double s1, double s2){ return Point{sin(atan(-1)) * s1, cos(atan(-1)) * s2}; },
        [](double s1, double s2){ return Point{sin(atan(-1 / 2.)) * s1, cos(atan(-1 / 2.)) * s2}; },
        [](double s1, double s2){ return Point{sin(atan(-1 / 3.)) * s1, cos(atan(-1 / 3.)) * s2}; },
        [](double s1, double s2){ return Point{0, s2}; },
        [](double s1, double s2){ return Point{sin(atan(1 / 3.)) * s1, cos(atan(1 / 3.)) * s2}; },
        [](double s1, double s2){ return Point{sin(atan(1 / 2.)) * s1, cos(atan(1 / 2.)) * s2}; },
        [](double s1, double s2){ return Point{sin(atan(1)) * s1, cos(atan(1)) * s2}; },
        [](double s1, double s2){ return Point{sin(atan(2)) * s1, cos(atan(2)) * s2}; },
        [](double s1, double s2){ return Point{sin(atan(3)) * s1, cos(atan(3)) * s2}; },
        [](double s1, double s2){ return Point{field_sett::max_field_width, field_sett::max_field_height}; }
    };
    double last_dist = field_sett::max_field_width;
    for (int i = 0; i < kPoint_offset; i++) {
        Point bp = point_offset[i](field_sett::parking_zone_door_size, field_sett::parking_zone_door_size) + p1;
        double dist = p2.dist(bp);
        if (dist >= last_dist) {
            parking_zone_circles_ = std::make_pair(p1, point_offset[(i - 1 + kPoint_offset) % kPoint_offset](field_sett::parking_zone_door_size, field_sett::parking_zone_door_size) + p1);
            delete_from_death_zone_circle(parking_zone_circles_.first);
            delete_from_death_zone_circle(parking_zone_circles_.second);
            auto p_off = point_offset[(i + kPoint_offset - 1) % kPoint_offset](field_sett::parking_zone_width_min, field_sett::parking_zone_width_min);
            parking_zone_back_.first = parking_zone_circles_.first + Point{p_off.get_y(), -p_off.get_x()};
            parking_zone_back_.second = parking_zone_circles_.second + Point{p_off.get_y(), -p_off.get_x()};
            // добавляем в контура
            borders.emplace_back();
            p_off = point_offset[(i + kPoint_offset - 1) % kPoint_offset](robot_sett::move_offset, robot_sett::move_offset);
            borders.back().push_back(parking_zone_circles_.first - p_off - Point{p_off.get_y(), -p_off.get_x()});
            borders.back().push_back(parking_zone_back_.first - p_off + Point{p_off.get_y(), -p_off.get_x()});
            borders.back().push_back(parking_zone_back_.second + p_off + Point{p_off.get_y(), -p_off.get_x()});
            borders.back().push_back(parking_zone_circles_.second + p_off - Point{p_off.get_y(), -p_off.get_x()});
            return;
        }
        last_dist = dist;
    }
    std::cerr << "Create Map by Point: not found same point!" << std::endl;
}

Map::Map(const Point &p_1, const Point &p_2) { // первая точка - правая точка
    add_pz(p_1, p_2);
}

Map::Map(const Point &p_1, const Point &p_2, const std::array<BoxMap, 3> &boxes, const RobotPoint &p) {
    add_pz(p_1, p_2);
    for (auto i : boxes) {
        add_box(i.get_left_corner_point());
        auto p_offset = get_field_unit(i.get_box_indent() - (field_sett::climate_box_offset / 2.));
        for (int x = std::max(0, p_offset.first); x < std::min(p_offset.first + field_sett::climate_box_number_unit_offset, field_sett::number_field_unit); x++) {
            for (int y = std::max(0, p_offset.second); y < std::min(p_offset.second + field_sett::climate_box_number_unit_offset, field_sett::number_field_unit); y++) {
                death_zone_[x][y] = false;
            }
        }
    }
    normal_death_zone();
    position_ = p;
}

std::vector<Point> get_corners_between_lines(const Point &a, const Point &b) {
    auto nearest_line = [](const Point &p) {
        double min = p.get_x();
        field_margin mr = left_field_margin;
        if (min > p.get_y()) {
            mr = top_field_margin;
            min = p.get_y();
        }
        if (min > fabs(field_sett::max_field_width - p.get_x())) {
          mr = right_field_margin;
          min = fabs(field_sett::max_field_width - p.get_x());
        }
        if (min > fabs(field_sett::max_field_height - p.get_y())) {
          mr = bottom_field_margin;
        }
        return mr;
    };
    const Point corner[4] = {{0, field_sett::max_field_height},
                             {0, 0},
                             {field_sett::max_field_width, 0},
                             {field_sett::max_field_width, field_sett::max_field_height}};
    field_margin end = nearest_line(a);
    std::vector<Point> ans;
    for (field_margin i = nearest_line(b); i != end; i = (field_margin)(((int)i - 1 + 4) % 4)) {
        ans.push_back(corner[(int)i]);
    }
    return ans;
}

const Point type_box_corner2offset[4] = {{0, 0},
                                         {-field_sett::climate_box_width, 0},
                                         {-field_sett::climate_box_width, field_sett::climate_box_height},
                                         {0, field_sett::climate_box_height}};

bool Map::add_box_from_line(std::vector<std::vector<std::pair<Point, line_t>>> &points, int i, int j) {
    box_corner_type type = get_box_corner_type(points[i], j);
    if (type == undef_bt) {
        return false;
    }
    Point offset_cub = type_box_corner2offset[type];
    if ((j + 1) < points[i].size()) {
        add_boxes_in_robot_pos(points[i][j].first + offset_cub,
                               points[i][j + 1].first + offset_cub);
    }
    if ((j - 1) >= 0) {
        add_boxes_in_robot_pos(points[i][j].first + offset_cub,
                               points[i][j - 1].first + offset_cub);
    }
    return true;
}

//std::vector<std::vector<Point>> death_outline;
void Map::lines_detection(const std::vector<std::vector<std::pair<Point, line_t>>> &points, show_img_debug debug) {
    const long long angle_accuracy = 1e6;

    RobotPoint robot_position;
    double start_angel = 0;
    size_t counter_parallel = 1;

    // Поиск нулевой линии
    double min_dist_x = field_sett::max_field_width;
    std::pair<int, int> min_dist_x_ind;
    bool is_fiend_start_line = false;
    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < (points[i].size() - 1); j++) {
            if ((points[i][j].second == border_lt) || (points[i][j].second == box_lt)){
                if ((points[i][j].first.get_x() < 0) && (points[i][j + 1].first.get_x()) >= 0) {
                    is_fiend_start_line = true;
                    min_dist_x_ind = std::make_pair(i, j);
                    break;
                }
                if (min_dist_x > fabs(points[i][j].first.get_x())) {
                    min_dist_x = fabs(points[i][j].first.get_x());
                    min_dist_x_ind = std::make_pair(i, j);
                }
            }
        }
        if (is_fiend_start_line) {
            break;
        }
    }

    start_angel = atan2(
        points[min_dist_x_ind.first][min_dist_x_ind.second + 1].first.get_y() - points[min_dist_x_ind.first][min_dist_x_ind.second].first.get_y(),
        points[min_dist_x_ind.first][min_dist_x_ind.second + 1].first.get_x()
            - points[min_dist_x_ind.first][min_dist_x_ind.second].first.get_x());
    if (start_angel > M_PI_2) {
        start_angel -= M_PI;
    } else if (start_angel < -M_PI_2) {
        start_angel += M_PI;
    }

    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < (points[i].size() - 1); j++) {
            if ((points[i][j].second == border_lt) || (points[i][j].second == box_lt)) {
                double angel = atan2(
                    points[i][j + 1].first.get_y() - points[i][j].first.get_y(),
                    points[i][j + 1].first.get_x()
                        - points[i][j].first.get_x());
                counter_parallel++;
                if (!is_parallel(start_angel, angel)) {
                    angel -= M_PI_2;
                }
                angel = std::fmod(std::fmod(angel, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI;
                if (angel > M_PI_2) {
                    angel -= M_PI;
                } else if (angel < -M_PI_2) {
                    angel += M_PI;
                }
                start_angel = ((start_angel * (counter_parallel - 1)) + angel)
                    / counter_parallel;
            }
        }
    }

    auto rot_points = points;

    Point min_point(field_sett::max_field_width, field_sett::max_field_height);
    for (int i = 0; i < rot_points.size(); i++) {
        for (int j = 0; j < rot_points[i].size(); j++) {
            if (min_point.get_x() > rot_points[i][j].first.get_x()) {
                min_point.set_x(rot_points[i][j].first.get_x());
            }
            if (min_point.get_y() > rot_points[i][j].first.get_y()) {
                min_point.set_y(rot_points[i][j].first.get_y());
            }
        }
    }

    if (debug != nullptr) {
        DebugFieldMat img;
        add_lines_img(img, points);
        add_point_img(img, min_point);
        debug("", img);
    }

    Point rot_zero = min_point;
    rot_zero.rotation(-start_angel);
    for (int i = 0; i < rot_points.size(); i++) {
        for (int j = 0; j < rot_points[i].size(); j++) {
            rot_points[i][j].first -= min_point;
            rot_points[i][j].first.rotation(-start_angel);
            rot_points[i][j].first += rot_zero;
        }
    }

    for (int i = 0; i < rot_points.size(); i++) {
        for (int j = 0; j < (rot_points[i].size() - 1); j++) {
            if (rot_points[i][j].second == border_lt) {
                PolarPoint p1 = rot_points[i][j].first.to_polar();
                PolarPoint p2 = rot_points[i][j + 1].first.to_polar();
                p1.set_f(M_PI - p1.get_f());
                p2.set_f(M_PI - p2.get_f());
                field_margin type;
                if (is_parallel(atan2(rot_points[i][j + 1].first.get_y() - rot_points[i][j].first.get_y(),
                                      rot_points[i][j + 1].first.get_x() - rot_points[i][j].first.get_x()), 0)) {
                    if (rot_points[i][j].first.get_y() > 0) {
                        type = top_field_margin;
                    } else {
                        type = bottom_field_margin;
                    }
                } else {
                    if (rot_points[i][j].first.get_x() > 0) {
                        type = right_field_margin;
                    } else {
                        type = left_field_margin;
                    }
                }
                RobotPoint p = get_coordinates_from_line(rot_points[i][j].first.dist(),
                                                    rot_points[i][j + 1].first.dist(), p2.get_f()-p1.get_f(), p1.get_f(), type);
                robot_position.merge(p);

            }
        }
    }

    robot_position.set_angle((start_angel)); //TODO: тут не уверен
    if (debug != nullptr) {
        DebugFieldMat img2;
        add_lines_img(img2, rot_points, true);
        add_robot_img_global(img2, robot_position);
        add_point_img(img2);
        show_debug_img("2", img2);
    }

    position_ = robot_position;

    int start_i = -1;
    int start_j = -1;
    for (int i = 0; i < rot_points.size(); i++) {
        for (int j = 0; j < rot_points[i].size(); j++) {
            if (rot_points[i][j].second == border_lt) {
                start_i = i;
                start_j = j;
                for (start_j;(start_j < rot_points[i].size()) && (rot_points[i][start_j].second == border_lt); start_j++) {}
                start_j--;
                break;
            }
        }
        if (start_i != -1) {
            break;
        }
    }

    for (int i = 0; i < death_zone_.size(); i++) {
        std::fill(death_zone_[i].begin(), death_zone_[i].end(), false);
    }
    std::vector<std::vector<Point>> death_outline;
    if (start_i != -1) {
        auto add_end = [&](int i, int j) {
          death_outline.back().push_back(rot_points[i][j].first * Point(1, -1) + robot_position);
          std::vector<Point> corn = get_corners_between_lines(death_outline.back().front(), death_outline.back().back());
          for (auto i : corn) {
              death_outline.back().push_back(i);
          }
        };
        const Point box_size(field_sett::climate_box_width, field_sett::climate_box_height);
        std::pair<Point, line_t> last = rot_points[start_i][start_j];
        bool stop_f = true;
        bool start_f = true;
        for (int i = start_i; stop_f; i = ((++i) % rot_points.size())) {
            for (int j = ((start_f) ? (start_j + 1) : 0); j < rot_points[i].size(); j++) {
                //std::cout << i << " " << j << std::endl;
                start_f = false;
                if ((start_i == i) && (start_j == j)) {
                    stop_f = false;
                    break;
                }
                if ((last.second != border_lt) || (rot_points[i][j].second != border_lt)) {
                    if (last.second == border_lt) {
                        death_outline.emplace_back();
                    } else {
                        death_outline.back().push_back(last.first * Point(1, -1) + robot_position);
                        if (rot_points[i][j].second == border_lt) {
                            add_end(i, j);
                        }
                    }
                }
                last = rot_points[i][j];
            }
        }
        if (last.second != border_lt) {
            death_outline.back().push_back(last.first * Point(1, -1) + robot_position );
            add_end(start_i, start_j);
        }
        for (auto i : death_outline) {
            set_death_outline(i);
        }
    }

    for (int i = 0; i < rot_points.size(); i++) {
        bool used_parking_zone = false;
        for (int j = 0; j < rot_points[i].size(); j++) {
            if (rot_points[i][j].second == parking_lt) {
                if (!used_parking_zone) {
                    used_parking_zone = true;
                    parking_detected_line_.emplace_back();
                }
                parking_detected_line_.back().push_back(rot_points[i][j].first);
            }else if (rot_points[i][j].second == box_lt) {
                add_box_from_line(rot_points, i, j);
            }
        }
    }
}

std::array<Point, 4> Map::get_parking_zone() const {
    return {parking_zone_circles_.first, parking_zone_circles_.second, parking_zone_back_.first, parking_zone_back_.second};
}

cv::Mat Map::get_img(int width, int height) {
    cv::Mat img(width, height, CV_8UC3);
    cv::rectangle(img, {0, 0}, {width, height}, cv::Scalar(255, 255, 255), cv::FILLED);
    const Point step_markup(width / field_sett::number_field_unit, height / field_sett::number_field_unit);
    const Point size_line(step_markup.get_x() / 10, step_markup.get_y() / 10);
    const cv::Scalar color_line(172,172,172);
    for (int i = 0; i <= field_sett::number_field_unit; i++) {
        for (int j = 0; j <= field_sett::number_field_unit; j++) {
            cv::line(img, {int(i * step_markup.get_x() - size_line.get_x()), int(j * step_markup.get_y())}, {int(i * step_markup.get_x() + size_line.get_x()), int(j * step_markup.get_y())}, color_line, 1);
            cv::line(img, {int(i * step_markup.get_x()), int(j * step_markup.get_y() - size_line.get_y())}, {int(i * step_markup.get_x()), int(j * step_markup.get_y() + size_line.get_y())}, color_line, 1);
        }
    }
    const int count_line = 5;
    const Point step_markup_line(width / count_line, height / count_line);
    for (int i = 0; i < count_line; i++) {
        cv::line(img, {int(i * step_markup_line.get_x()), 0}, {int(i * step_markup_line.get_x()), width}, color_line, 1);
        cv::line(img, {0, int(i * step_markup_line.get_y())}, {height, int(i * step_markup_line.get_y())}, color_line, 1);
    }
    const Point zoom(double(width) / field_sett::max_field_width, double(height) / field_sett::max_field_height);
    auto b = get_boxes_normal();
    for (int i = 0; i < box_count_; i++) {
        Point p(b[i] * zoom);
        cv::rectangle(img, {int(p.get_x()), int(p.get_y())}, {int(p.get_x() + field_sett::climate_box_width * zoom.get_x()), int(p.get_y() + field_sett::climate_box_height * zoom.get_y())}, box_color2color[boxes_[i].get_color()], cv::FILLED);
    }
    const double radius_robot_line = 15;
    cv::circle(img, {int(position_.get_x() * zoom.get_x()), int(position_.get_y() * zoom.get_y())}, 10, {0, 0, 0}, cv::FILLED);
    cv::line(img, {int(position_.get_x() * zoom.get_x()), int(position_.get_y() * zoom.get_y())},
             {int((position_.get_x() * zoom.get_x()) + radius_robot_line * cos(M_PI - position_.get_angle()) ),
              int((position_.get_y() * zoom.get_y()) - radius_robot_line * sin(M_PI - position_.get_angle()))}, {0, 0, 255}, 3);
    std::vector<std::vector<cv::Point>> contours; //{{cv::Point(0, 10000), cv::Point(10, 10), cv::Point(0, 0)}};
    ///
//    for (auto i : death_outline) {
//        contours.emplace_back();
//        for (auto j : i) {
//            contours.back().push_back({int(j.get_x() * zoom.get_x()), int(j.get_y() * zoom.get_y())});
//        }
//    }
//    for (int i = 0; i < contours.size(); i++) {
//        cv::drawContours( img, contours, i, {255, 0, 0}, CV_FILLED);
//        show_debug_img("", img);
//    }
    ///
    for (int i = 0; i < death_zone_.size(); i++) {
        for (int j = 0; j < death_zone_[i].size(); j++) {
            if (death_zone_[i][j]) {
                cv::rectangle(img, {int(i * field_sett::size_field_unit * zoom.get_x()), int(j * field_sett::size_field_unit * zoom.get_y())}, {int((i + 1) * field_sett::size_field_unit * zoom.get_x()), int((j + 1) * field_sett::size_field_unit * zoom.get_y())}, {0, 0, 0}, cv::FILLED);
            }
        }
    }
    for (int i = 0; i < borders.size(); i++) {
        for (int j = 0; j < borders[i].size(); j++) {
            cv::line(img, {int(borders[i][j].get_x() * zoom.get_x()), int(borders[i][j].get_y() * zoom.get_y())},
                     {int(borders[i][(j + 1) % borders[i].size()].get_x() * zoom.get_x()), int(borders[i][(j + 1) % borders[i].size()].get_y() * zoom.get_y())}, {0, 128, 255}, 3);
        }
    }
    return img;
}

std::pair <int, int> Map::get_field_unit(const Point &p) {
    int n_x = fdiv(p.get_x() + (field_sett::size_field_unit / 2), field_sett::size_field_unit);
    int n_y = fdiv(p.get_y() + (field_sett::size_field_unit / 2), field_sett::size_field_unit);
    return std::make_pair(std::min(n_x, field_sett::number_field_unit - 1),
                          std::min(n_y, field_sett::number_field_unit - 1));
}

Point Map::normal_point(const Point &p) {
    auto n = get_field_unit(p);
    return Point(field_sett::size_field_unit * n.first,
                 field_sett::size_field_unit * n.second);
}

std::vector<Point> Map::get_boxes_normal() const {
    std::vector<Point> arr;
    for (int i = 0; i < box_count_; i++) {
        arr.push_back(normal_point(boxes_[i].get_left_corner_point()));
    }
    return arr;
}

std::array<std::array<bool, field_sett::number_field_unit>,
           field_sett::number_field_unit> Map::get_death_zone() const {
    return death_zone_;
}

std::vector<std::vector<Point>> Map::get_parking_zone_line() const {
    return parking_detected_line_;
}

Map::Map(const std::vector<std::vector<std::pair<Point, line_t>>> &p, show_img_debug debug) {
    lines_detection(p, debug);
}
Map::Map(const std::vector<PolarPoint> &p, show_img_debug debug) {
    lines_detection(detected_lines_and_types(p), debug);
}

std::pair<int, double> Map::get_nearby_box(const Point &p) {
    std::pair<int, double> ans = std::make_pair(-1, field_sett::max_field_width);
    for (int i = 0; i < boxes_.size(); i++) {
        if (ans.second > p.dist(boxes_[i].get_left_corner_point())) {
            ans.first = i;
            ans.second = p.dist(boxes_[i].get_left_corner_point());
        }
    }
    return ans;
}

Point right_turn(const Point &p, const Point &offset) {
    return {offset.get_x() - p.get_y(), p.get_x()};
}

Point double_turn(const Point &p, const Point &offset) {
    Point a = p - offset;
    return {-a.get_x() + offset.get_x(), -a.get_y() + offset.get_y()};
}

Point left_turn(const Point &p, const Point &offset) {
    return {p.get_y(), offset.get_y() - p.get_x()};
}

MassPoint mass_save_turn(const MassPoint &p, Point (* turn)(const Point &p, const Point &offset), const Point &offset) {
    auto mass = p.get_mass();
    MassPoint ans = turn(p, offset);
    ans.set_mass(mass);
    return ans;
}

std::pair<int, int> right_turn_death_zone(const std::pair<int, int> &p,
                                    int n = field_sett::number_field_unit) {
    return std::make_pair(n - p.second - 1, p.first);
}

std::pair<int, int> double_turn_death_zone(const std::pair<int, int> &p,
                                          int n = field_sett::number_field_unit) {
    return std::make_pair(p.first, n - p.second - 1);
}

std::pair<int, int> left_turn_death_zone(const std::pair<int, int> &p,
                                          int n = field_sett::number_field_unit) {
    return std::make_pair(p.second, n - p.first - 1);
}

void Map::turn(int a) {
    if ((a < 0 ) || (a > 3)) {
        std::cerr << "Map::turn:: Wrong boundaries: expect 0 <= a <= 3; a = "
                  << a << std::endl;
    }
    if (a == 0)
        return;
    a--;
    Point (* turns[])(const Point &p, const Point &offset) = {right_turn, double_turn, left_turn};
    const Point boxes_offset[3] = {{-field_sett::climate_box_width, 0},
                                   {-field_sett::climate_box_width, -field_sett::climate_box_height},
                                   {0, -field_sett::climate_box_height}};
    Point offset_field(field_sett::max_field_width, field_sett::max_field_height);
    for (int i = 0; i < box_count_; i++) {
        boxes_[i].set_left_corner_point(mass_save_turn(boxes_[i].get_left_corner_point(), turns[a], offset_field) + boxes_offset[a]);
    }
    parking_zone_circles_.first = turns[a](parking_zone_circles_.first, offset_field);
    parking_zone_circles_.second = turns[a](parking_zone_circles_.second, offset_field);
    parking_zone_back_.first = turns[a](parking_zone_back_.first, offset_field);
    parking_zone_back_.second = turns[a](parking_zone_back_.second, offset_field);
    for (int i = 0; i < parking_detected_line_.size(); i++) {
        for (int  j = 0; j < parking_detected_line_[i].size(); j++) {
            parking_detected_line_[i][j] = (turns[a](parking_detected_line_[i][j], offset_field));
        }
    }
    std::pair<int, int> (* turns_dz[])(const std::pair<int, int> &p, int n) = {right_turn_death_zone, double_turn_death_zone, left_turn_death_zone};
    auto dz = death_zone_;
    for (int i = 0; i < death_zone_.size(); i++) {
        for (int j = 0; j < death_zone_[i].size(); j++) {
            std::pair<int, int> new_ind = turns_dz[a](std::make_pair(i, j), death_zone_.size());
            death_zone_[new_ind.first][new_ind.second] = dz[i][j];
        }
    }
    Point pos = turns[a](position_, offset_field);
    position_.set_x(pos.get_x());
    position_.set_y(pos.get_y());
    position_.add_angle(M_PI_2 * (a + 1));
}

bool Map::in_death_zone(const Point &point) {
    auto ind = get_field_unit(point);
    return death_zone_[ind.first][ind.second];
}

bool Map::merge(const Map &a) {
    //TODO: учесть парковку
    Map m = a;
    auto normal_boxes = get_boxes_normal();
    size_t count_same = 0;
    std::pair<int, std::set<int>> same;
    std::set<int> used_boxes;
    for (int i = 0; i < 4; i++) {
        bool same_all = true;
        for (auto j : normal_boxes) {
            auto nearly = m.get_nearby_box(j);
            bool in_death = m.in_death_zone(j);
            if ((nearly.second > (field_sett::size_field_unit / 2.)) && !in_death) {
                same_all = false;
                break;
            }
            if (!in_death) {
                used_boxes.insert(nearly.first);
            }
        }
        auto m_normal_box = m.get_boxes_normal();
        for (int j = 0; (j < m_normal_box.size()) && same_all; j++) {
            if ((used_boxes.find(j) == used_boxes.end()) && !in_death_zone(m_normal_box[j])) {
                same_all = false;
                break;
            }
        }
        if (same_all) {
            count_same++;
            same.first = i;
            same.second = used_boxes;
        }
        show_debug_img("", m.get_img());
        m.turn();
    }
    if (count_same != 1) {
        return false;
    }
    m.turn(same.first);
    position_ = m.position_;
    for (int i = 0; i < m.box_count_; i++) {
        add_box(m.boxes_[i].get_left_corner_point());
        //boxes_[i].merge(m.boxes_[m.get_nearby_box(boxes_[i].get_left_corner_point()).first]);
    }
    for (int i = 0; i < death_zone_.size(); i++) {
        for (int j = 0; j < death_zone_[i].size(); j++) {
            death_zone_[i][j] = death_zone_[i][j] && m.death_zone_[i][j];
        }
    }
    return true;
}

bool Map::death_rect(const cv::Point2i &a, const cv::Point2i &b) {
    cv::Point2i max(std::max(a.x, b.x), std::max(a.y, b.y));
    cv::Point2i min(std::min(a.x, b.x), std::min(a.y, b.y));
    if ((min.x < 0) || (max.x >= death_zone_.size()) || (min.y < 0) || (max.y >= death_zone_.front().size())) {
        return false;
    }
    bool ans = true;
    for (int i = min.x; i <= max.x; i++) {
        for (int j = min.y; j <= max.y; j++) {
            ans = ans && death_zone_[i][j];
        }
    }
    return ans;
}

void Map::normal_death_zone() {
    const cv::Point2i unit_size_box
        (std::ceil(field_sett::climate_box_width / field_sett::size_field_unit - 1),
         std::ceil(field_sett::climate_box_height / field_sett::size_field_unit - 1));
    for (int i = 0; i < death_zone_.size(); i++) {
        for (int j = 0; j < death_zone_[i].size(); j++) {
            if ((i == 4) && (j == 7)) {
                std::cout << " ";
            }
            death_zone_[i][j] =
                   death_rect({i, j}, {i + unit_size_box.x, j + unit_size_box.y})
                || death_rect({i, j}, {i - unit_size_box.x, j + unit_size_box.y})
                || death_rect({i, j}, {i + unit_size_box.x, j - unit_size_box.y})
                || death_rect({i, j}, {i - unit_size_box.x, j - unit_size_box.y});
        }
    }
}

void Map::update(const std::vector<PolarPoint> &polar_points, show_img_debug debug) {
    std::vector<std::vector<Point>> points = get_corners(polar_points);
    std::vector<std::vector<Point>> points_in_robot;
    double ang_offset = - position_.get_angle();
    for (int i = 0; i < points.size(); i++) {
        points_in_robot.emplace_back();
        for (int j = 0; j < points[i].size(); j++) {
            Point new_point = {points[i][j].get_x() * cos(ang_offset) - points[i][j].get_y() * sin(ang_offset),
                               points[i][j].get_x() * sin(ang_offset) + points[i][j].get_y() * cos(ang_offset)};
            points_in_robot.back().push_back(new_point);
            points[i][j] = (new_point + Point(position_.get_x(), -position_.get_y())) * Point{1, -1};
        }
    }
    double a = PolarPoint::angle_norm(
        lidar_sett::ang_death_start
            + position_.get_angle());
    double b = PolarPoint::angle_norm(
        lidar_sett::ang_death_end
            + position_.get_angle());
    auto p = std::make_pair(a, b);
    std::vector<std::vector<std::pair<Point, line_t>>> lines;
    line_detect_from_pos(lines, parking_zone_circles_,
                         points, p, position_);
    if (debug != nullptr) {
        DebugFieldMat mat1;
        add_lines_img(mat1, lines, true);
        debug("Update_map_in_global", mat1);
    }
    MassPoint new_pos;
    for (auto i : lines) {
        for (int j = 0; j < i.size() - 1; j++) {
            if (i[j].second == border_lt) {
                MassPoint buff_p;
                if (fabs(i[j].first.get_x() - i[j + 1].first.get_x()) < fabs(i[j].first.get_y() - i[j + 1].first.get_y())) {
                    if (i[j].first.get_y() > position_.get_y()) {
                        buff_p.set_x(dist_line2point(i[j].first,
                                                     i[j + 1].first,
                                                     position_));
                    } else {
                        buff_p.set_x(field_sett::max_field_width -
                                     dist_line2point(i[j].first,
                                                     i[j + 1].first,
                                                     position_));
                    }
                } else {
                    if (i[j].first.get_x() < position_.get_x()) {
                        buff_p.set_y(dist_line2point(i[j].first,
                                                     i[j + 1].first ,
                                                     position_));
                    } else {
                        buff_p.set_y(field_sett::max_field_height -
                            dist_line2point(i[j].first,
                                            i[j + 1].first,
                                            position_));
                    }
                }
                new_pos.merge(buff_p);
            }
        }
    }
    Point offset2new_position = new_pos - position_;
    position_.set_x(new_pos.get_x());
    position_.set_y(new_pos.get_y());
    for (auto i : lines) {
        for (auto j : i) {
            j.first += offset2new_position;
        }
    }
    delete_from_death_zone_circle(position_, lidar_sett::max_visible_black);
    for (int i = 0; i < lines.size(); i++) {
        std::vector<Point> outline;
        for (int j = 0; j < lines[i].size(); j++) {
            outline.push_back(lines[i][j].first);
        }
        outline.push_back(position_);
        set_death_outline(outline, false);
    }
    // Переводим в систему координат робота
    for (int i = 0; i < lines.size(); i++) {
        for (int j = 0; j < lines[i].size(); j++) {
            Point buff = Point{1, -1} * (lines[i][j].first - position_);
            lines[i][j].first = buff;
        }
    }
    normal_death_zone();
    if (debug != nullptr) {
        DebugFieldMat mat1;
        add_lines_img(mat1, lines, true);
        debug("Update_map_in_robot", mat1);
    }
    for (int i = 0; i < lines.size(); i++) {
        for (int j = 0; j < lines[i].size(); j++) {
            if (lines[i][j].second == box_lt) {
                add_box_from_line(lines, i, j);
            }
        }
    }
    if (debug != nullptr) {
        debug("After_update_map", get_img());
    }
}
