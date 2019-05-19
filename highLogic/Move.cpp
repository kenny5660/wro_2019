//
// Created by Danila on 19.04.2019.
//

#include "Move.h"
#include "lidar_math.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

enum move_unit_t {
    wall_t,
    undif_t,
    step_t,
    unused_t
};

typedef std::array<std::array<move_unit_t, height_mesh + 1>, width_mesh +1> mesh_t;

std::string mesh2string(const mesh_t &m) {
    std::string s = "";
    for (int j = 0; j < m.front().size(); j++) {
        for (int i = 0; i < m.size(); i++) {
            s += std::to_string(m[i][j]) + " ";
        }
        s += "\n";
    }
    s.pop_back();
    s.pop_back();
    return s;
}

cv::Point2i map_point2move_unit(const Point &p) {
    return cv::Point2i(fdiv(p.get_x(),
                           field_sett::max_field_width / (double) width_mesh),
                       fdiv(p.get_y(),
                           field_sett::max_field_height / (double)height_mesh));
}

inline bool mesh_point_check(mesh_t &m, const cv::Point &p) { // return true if all ok
    return ((p.x >= 0) && (p.x < m.size()) && (p.y >= 0) && (p.y < m.begin()->size()));
}

void set_mesh(mesh_t &m, const cv::Point &p, move_unit_t t) {
    if (mesh_point_check(m, p)) {
        if (m[p.x][p.y] > t) {
            m[p.x][p.y] = t;
        }
    }
}

void add_point(mesh_t &m, const cv::Point &p,  move_unit_t t) {
    for (int i = -unit_offset + 1; i <= unit_offset; i++) {
        for (int j = -unit_offset + 1; j <= unit_offset; j++) {
            set_mesh(m, {p.x + i, p.y + j}, t);
        }
    }
}

void add_rect(mesh_t &m, const cv::Point &p1, const cv::Point &p2,  move_unit_t t) {
    cv::Point min(std::min(p1.x, p2.x), std::min(p1.y, p2.y));
    cv::Point max(std::max(p1.x, p2.x), std::max(p1.y, p2.y));
    for (int i = min.x; i <= max.x; i++) {
        for (int j = min.y; j <= max.y; j++) {
            add_point(m, {i, j}, t);
        }
    }
}

void add_box(mesh_t &m, const Point &box) {
    const double round_offset = 5; // нормализованные коробки при округлении попадают не в тот квадрат и их нужно слегка сдвинуть
    cv::Point p1 = map_point2move_unit(box + round_offset);
    cv::Point p2 = map_point2move_unit(box + Point(field_sett::climate_box_width, field_sett::climate_box_height) + round_offset);
    add_rect(m, p1, p2, wall_t);
}

mesh_t get_mesh(const Map &map) {
    mesh_t mesh;
    for (int i = 0; i < mesh.size(); i++) {
        for (int j = 0; j < mesh.front().size(); j++) {
            mesh[i][j] = unused_t;
        }
    }
    for (int i = 0; i < mesh.size(); i++) {
        for (int j = 0; j < unit_offset; j++) {
            mesh[i][j] = wall_t;
        }
    }
    for (int i = 0; i < mesh.size(); i++) {
        for (int j = mesh.front().size() - unit_offset; j < mesh.front().size(); j++) {
            mesh[i][j] = wall_t;
        }
    }
    for (int i = 0; i < unit_offset; i++) {
        for (int j = 0; j < mesh.front().size(); j++) {
            mesh[i][j] = wall_t;
        }
    }
    for (int i = mesh.size() - unit_offset; i < mesh.size(); i++) {
        for (int j = 0; j < mesh.front().size(); j++) {
            mesh[i][j] = wall_t;
        }
    }
    // Добавляем мёртвую зону
    auto death_zone = map.get_death_zone();
    cv::Point k_unit(mesh.size() / (death_zone.size() - 1), mesh.front().size() / (death_zone.front().size() - 1));
    for (int i = 0; i < mesh.size(); i++) {
        for (int j = 0; j < mesh.front().size(); j++) {
            if (death_zone[i / k_unit.x][j / k_unit.y]) {
                add_point(mesh, {i, j}, undif_t);
            }
        }
    }
    // Добавляем коробки
    auto boxes = map.get_boxes_normal();
    for (auto i : boxes) {
        add_box(mesh, i);
    }
    //TODO:: add parking_zone по нормальному, а не по тупому (не тупо квадратом)
    auto pz_point = map.get_parking_zone();
    if (!std::isnan(pz_point[0].get_x()) && !std::isnan(pz_point[0].get_y())) {
        cv::Point pz_min = map_point2move_unit(pz_point[0] + 10);
        cv::Point pz_max = map_point2move_unit(pz_point[0] + 10);
        for (auto i : pz_point) {
            auto a = map_point2move_unit(i + 10);
            pz_min.x = std::min(pz_min.x, a.x);
            pz_min.y = std::min(pz_min.y, a.y);
            pz_max.x = std::max(pz_max.x, a.x);
            pz_max.y = std::max(pz_max.y, a.y);
        }
        add_rect(mesh, {pz_min.x - unit_offset + 1, pz_min.y - unit_offset + 1},
                 {pz_max.x + unit_offset, pz_max.y + unit_offset}, wall_t);
    }
    std::cout << mesh2string(mesh) << std::endl;
    return mesh;
}

cv::Point point2corner(mesh_t m, const Point &point) {
    auto buff_p = Map::get_field_unit(point);
    cv::Point p(buff_p.first, buff_p.second);
    if ((!mesh_point_check(m, p)) || (m[p.x][p.y] == wall_t)) {
        std::cerr << "Move::point2corner:\n Detektion busy point!" << std::endl;
        bool is_found = false;
        p = map_point2move_unit(point);
        for (int i = 1 - unit_offset; i <= unit_offset; i++) {
            for (int j = 1 - unit_offset; j <= unit_offset; j++) {
                if (((!mesh_point_check(m, {p.x + i, p.y + j})) || (m[p.x + i][p.y + j] == wall_t))) {
                    is_found = true;
                    p.x += i;
                    p.y += j;
                    break;
                }
            }
            if (is_found) {
                break;
            }
        }
    }
    if ((!mesh_point_check(m, p)) || (m[p.x][p.y] == wall_t)) {
        std::cerr << "Move::point2corner:\n Pizdec! Iato krai!" << std::endl;
        return {0, 0};
    }
    return p;
}

const std::array<std::array<int, 3>, 3> mov_step = {{
    {-1, 0, 1},
    {0, 1, -1},
    {1, 0, -1}}
};

void bfs(mesh_t &m, const cv::Point &now, const cv::Point &end, std::vector<cv::Point> &p, bool &is_found, move_unit_t stop_t = undif_t) {
    //std::cout << "X: " << now.x << " Y: " << now.y << std::endl;
    if (is_found || !mesh_point_check(m, now) || (m[now.x][now.y] <= stop_t) || (m[now.x][now.y] == step_t)) {
        return;
    }
    p.push_back(now);
    if ((now.x == end.x) && (now.y == end.y)) {
        is_found = true;
        return;
    }
    m[now.x][now.y] = step_t;
    int type_x = sign(end.x - now.x) + 1;
    int type_y = sign(end.y - now.y) + 1;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if ((mov_step[type_x][i] == 0) && (mov_step[type_y][j] == 0)) {
                    continue;
                }
                bfs(m,
                    cv::Point(now.x + ((type_x == 1) ? (mov_step[type_x][j]) : (mov_step[type_x][i])),
                              now.y + ((type_x == 1) ? (mov_step[type_y][i]) : (mov_step[type_y][j]))),
                    end,
                    p,
                    is_found,
                    stop_t);
            }
        }
//    for (int i = (end.x > now.x) ? (1) : (-1);
//         (end.x > now.x) ? (i >= -1) : (i <= 1);
//         (end.x > now.x) ? (i--) : (i++)) {
//        for (int j = (end.y > now.y) ? (1) : (-1);
//             (end.y > now.y) ? (j >= -1) : (j <= 1);
//             (end.y > now.y) ? (j--) : (j++)) {
//            if ((i == 0) && (j == 0)) {
//                continue;
//            }
//            bfs(m,
//                cv::Point(now.x + i, now.y + j),
//                end,
//                p,
//                is_found,
//                stop_t);
//        }
//    }
    //Зеленский выйгпал выборы!;
    if (!is_found) {
        p.pop_back();
    }
}

bool go_to(Map &map, const Point &point, std::vector<Point> &ans, Point &end_point, bool kamikaze_mode, show_img_debug debug) {
    mesh_t mesh = get_mesh(map);
    cv::Point start = point2corner(mesh, map.get_position());
    cv::Point end = point2corner(mesh, point);
    std::vector<cv::Point> way;
    bool is_found = false;
    mesh_t mesh_buff = mesh;
    bool is_first_found;
    bfs(mesh_buff, start, end, way, is_found, (kamikaze_mode) ? (wall_t) : (undif_t));
    is_first_found = is_found;
    if (!is_found && !kamikaze_mode) {
        std::vector<cv::Point> buff_way;
        mesh_buff = mesh;
        bfs(mesh_buff, start, end, buff_way, is_found, wall_t);
        if (!is_found) {
            std::cerr << "Move::go_to:\n Any way not found!" << std::endl;
            return false;
        }
        for (auto i : buff_way) {
            if (mesh[i.x][i.y] < step_t) {
                break;
            }
            way.push_back(i);
        }
    }
    if (debug != nullptr) {
        cv::Mat img = map.get_img();
        for (int i = 0; i < way.size() - 1; i++) {
            cv::Point img_p1 = {(int)(way[i].x * (field_sett::size_field_unit * img.size().width / field_sett::max_field_width)),
                               (int)(way[i].y * (field_sett::size_field_unit * img.size().height / field_sett::max_field_height))};
            cv::Point img_p2 = {(int)(way[i + 1].x * (field_sett::size_field_unit * img.size().width / field_sett::max_field_width)),
                                (int)(way[i + 1].y * (field_sett::size_field_unit * img.size().height / field_sett::max_field_height))};
            cv::line(img, img_p1, img_p2, {55, 178, 7}, 3);
        }
        debug("way", img);
    }
	end_point = { way.back().x * field_sett::size_field_unit, way.back().y * field_sett::size_field_unit };
    ans.emplace_back(way.front().x * field_sett::size_field_unit
                           - map.get_position().get_x(),
                       way.front().y * field_sett::size_field_unit
                           - map.get_position().get_y());
    for (int i = 1; i < way.size(); i++) {
        ans.emplace_back((way[i].x - way[i - 1].x) * field_sett::size_field_unit,
                           (way[i].y - way[i - 1].y) * field_sett::size_field_unit);
    }
    for (int i = 0; i <ans.size(); i++) {
	    Point buff = ans[i];
	    ans[i].set_y(-buff.get_x());
	    ans[i].set_x(-buff.get_y());
    }
    double ang_off = map.get_position().get_angle();
    for (int i = 0; i <ans.size(); i++) {
        Point new_point(ans[i].get_x() * cos(ang_off) - ans[i].get_y() * sin(ang_off),
                        ans[i].get_x() * sin(ang_off) + ans[i].get_y() * cos(ang_off));
        ans[i] = new_point;
    }
    return is_first_found;
}
