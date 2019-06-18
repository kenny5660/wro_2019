//
// Created by Danila on 06.05.2019.
//

#include "alg.h"
#include "map.h"
#include "CV.h"
#include "Move.h"
#include "lidar_math.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <algorithm>

Point catch_flower_offset[4] = {
    {-robot_sett::catch_flower_offset, 0},
    {0, robot_sett::catch_flower_offset},
    {robot_sett::catch_flower_offset, 0},
	{0, -robot_sett::catch_flower_offset}
};

//TODO: анализ после мёртвой зоны

int turn2box(Robot &robot, BoxMap &box, Map &map) {
    int need_rot = 1;
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        show_debug_img("", map.get_img());
    #endif
    Point buff_p = box.get_box_indent();
    buff_p = buff_p - Point{field_sett::size_field_unit, field_sett::size_field_unit};
    if (fabs(buff_p.get_x() - box.get_left_corner_point().get_x()) < fabs(buff_p.get_y() - box.get_left_corner_point().get_y())) {
        if (box.get_left_corner_point().get_y() > buff_p.get_y()) {
            need_rot = 2;
        } else {
            need_rot = 0;
        }
    } else {
        if (box.get_left_corner_point().get_x() > buff_p.get_x()) {
            need_rot = 3;
        }
    }
    double need_rot_rad = (need_rot) * M_PI_2 + map.get_position().get_angle();
    {
        write_log("Need rood: \n" + std::to_string(need_rot_rad));
    }
    robot.Turn(need_rot_rad);
    map.set_new_position(RobotPoint{map.get_position().get_x(), map.get_position().get_y(),
                                    -need_rot_rad +  map.get_position().get_angle()});
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        show_debug_img("", map.get_img());
    #endif
    return need_rot;
}

void box_connect(Robot &robot) {
    std::vector<PolarPoint> lidar_data;
    robot.GetLidarPolarPoints(lidar_data);

    // TODO: !!! Если коробок больше одной передать первым параметром
    // длину, вторым какой нам нужен, а так же взять данные с камеры.

    Point center = position_box_side(lidar_data, 1, 1, {0, 0}, save_debug_img);
    robot.Go2({{-(field_sett::climate_box_offset / 2 - center.get_y()), 0}});
    robot.Go2({{0, -center.get_x()}});
}

void go2box(Robot &robot, std::vector<Point> way, Robot::CatchCubeSideEnum side_catch) {
    const double care_way_dist = field_sett::size_field_unit * 2;
    Point end_p = way.back();
    double dist = way.back().dist() - care_way_dist;
    double ang = atan2(way.back().get_y(), way.back().get_x());
    way.back() = {cos(ang) * dist, sin(ang) * dist};
    robot.Go2(way);

    std::vector<PolarPoint> lidar_data;
    robot.GetLidarPolarPoints(lidar_data);
    Point center = position_box_side(lidar_data, 1, 1, {-sin(ang) * care_way_dist - (field_sett::climate_box_width / 2.0), cos(ang) * care_way_dist + care_way_dist}, save_debug_img);
    center = {center.get_y() - care_way_dist, -center.get_x() - ((side_catch == Robot::CatchCubeSideEnum::LEFT) ? (field_sett::size_field_unit / 3) : (-10))};
    robot.Go2({{center}});
}

void frame_connect(Robot &robot, double out_way_offset, double start_angle) {
	robot.Turn(-start_angle);
	Point point_offset = { -field_sett::parking_zone_door_size / 2., out_way_offset };
	std::vector<PolarPoint> lidar_dt;
	robot.GetLidarPolarPoints(lidar_dt);
	auto points = get_corners(lidar_dt);
	std::pair<int, int> ind_nearly_point = { 0, 0 };
	for (int i = 1; i < points.size(); i++) {
		for (int j = 0; j < points[i].size(); j++) {
			if (points[ind_nearly_point.first][ind_nearly_point.second].dist(point_offset) >
			    points[i][j].dist(point_offset)) {
				ind_nearly_point = std::make_pair(i, j);
			}
		}
	}
	{
		DebugFieldMat mat;
		auto ln = line2line_type(points);
		add_lines_img(mat, ln);
		add_point_img(mat);
		add_point_img(mat, point_offset);
		save_debug_img("frame_conect", mat);
	}
	Point p = { points[ind_nearly_point.first][ind_nearly_point.second].get_y(), -points[ind_nearly_point.first][ind_nearly_point.second].get_x() - field_sett::parking_zone_door_size / 2.0 };
	robot.Go2({ { 0, p.get_y() } });
	robot.Go2({ { p.get_x() + 110, 0 } });
	robot.AlliginByDist(45, 0);
}

Point go_from_frame(Robot &robot, double dist, double ang) {
    robot.Go2({{0, dist}});
	robot.Turn(ang);
    return {cos(ang) * dist, sin(ang) * dist};
}

color_t do_box(Robot &robot, Map &map, BoxMap &box, Robot::CatchCubeSideEnum &side_catch,
                bool need_next, bool kamikaze_mode, show_img_debug db) {
    std::vector<Point> way;
    int need_rot = turn2box(robot, box, map);
    {
        write_log("Found way to: \n"
                  " x = " + std::to_string(box.get_box_indent().get_x()) +
            "\n y = " + std::to_string(box.get_box_indent().get_y()));
    }
    Point end_point;
    bool way_found = go_to2(map, box.get_box_indent(), way, end_point, kamikaze_mode, db);
    while (!way_found) {
        {
            save_debug_img("map_befor", map.get_img());
        }
        robot.Go2(way);
        map.set_new_position(RobotPoint{ end_point.get_x(), end_point.get_y(), map.get_position().get_angle() });
        double ang = M_PI - atan2(
            way[way.size() - 1].get_y() - way[way.size() - 2].get_y(),
            way[way.size() - 1].get_x() - way[way.size() - 2].get_x());
        ang = PolarPoint::angle_norm(ang) - map.get_position().get_angle();
        robot.Turn(ang);
        RobotPoint pos = map.get_position();
        pos.add_angle(-ang);
        map.set_new_position(pos);
        std::vector<PolarPoint> ld;
        robot.GetLidarPolarPoints(ld);
        map.update(ld, db);
        {
            save_debug_img("map_after", map.get_img());
        }
        robot.Turn(-ang);
        pos.add_angle(ang);
        map.set_new_position(pos);
        way_found = go_to2(map, box.get_box_indent(), way, end_point, false, db);
        if (!way_found && (way.size() == 2) &&
            ((fabs(way.front().get_x() - way.back().get_x())) < 10) &&
            ((fabs(way.front().get_x() - way.back().get_x())) < 10)) {
            //TODO: если нет путит
        }
    }
    {
        write_log("Way founded.");
    }
    go2box(robot, way, side_catch);
    map.set_new_position(RobotPoint{ end_point.get_x(), end_point.get_y(), map.get_position().get_angle() });
    color_t color_next = robot.CatchCube(side_catch, need_next);
    side_catch = (side_catch == Robot::CatchCubeSideEnum::LEFT) ?
                 (Robot::CatchCubeSideEnum::RIGHT) :
                 (Robot::CatchCubeSideEnum::LEFT);
    {
        write_log("Position after catch: \n"
                  " x = " + std::to_string(way.back().get_x()) +
            "\n y = " + std::to_string(way.back().get_y()));
    }
    return color_next;
}

void do_alg_code(Robot &robot, bool kamikaze_mode, std::string s) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	clear_logs();
#endif
    Robot::CatchCubeSideEnum side_catch = Robot::CatchCubeSideEnum::LEFT;
    cv::Mat QRCodeImg;
    if (s == "") {
        robot.WayFromFrame(QRCodeImg);
    }
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, s);
    double start_angle = start_position.get_angle();
    {
        write_log("Start angle: " + std::to_string(start_angle));
        std::cout << start_angle << std::endl;
    }
    {
        Point st_bf_off = go_from_frame(robot, out_way_offset, start_angle);
        start_position.set_x(start_position.get_x() - st_bf_off.get_x());
        start_position.set_y(start_position.get_y() - st_bf_off.get_y());
    }
    start_position.set_angle(0);
    Map map(pz.first, pz.second, boxes, start_position);
    {
        write_log(
            "Start position:\n x = " + std::to_string(start_position.get_x()) +
                "\n y = " + std::to_string(start_position.get_y()) +
                "\n ang = " + std::to_string(start_position.get_angle()));
        save_debug_img("Start_map", map.get_img());
    }
    start_position.set_angle(start_angle);
    save_debug_img("QRCodeDetection", map.get_img());
    std::vector<Point> way;
	show_img_debug db;	
	#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		db = show_debug_img;
	#else
		db = save_debug_img;
	#endif
	for (auto i : boxes) {
        do_box(robot, map, i, side_catch, false, kamikaze_mode, db);
    }
    way.clear();
    Point b;
	robot.Turn(map.get_position().get_angle() - M_PI_2);
	auto buff = map.get_position();
	buff.add_angle(-map.get_position().get_angle() + M_PI_2);
	map.set_new_position(buff);
    if(!go_to2(map, start_position, way, b, kamikaze_mode, db))
        return;
    robot.Go2(way);
    frame_connect(robot, out_way_offset, start_angle);
}

RobotPoint detect_position(Robot &robot, std::vector<PolarPoint> &lidar_data, double frame_offset) {
	//TODO: проверить, что стойки не мешают алгоритму.
  show_img_debug debug;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	debug = show_debug_img;
#else
	debug = save_debug_img;
#endif

	robot.GetLidarPolarPoints(lidar_data);
	auto lines = get_corners(lidar_data);
	double min_length = 2 * field_sett::size_field_unit;

	// Нужно развернуть систему координат.
	double ang = get_angle_lines(lines,
		{
			 { frame_offset, field_sett::parking_zone_door_size / 2. },
			{ frame_offset, -field_sett::parking_zone_door_size / 2. } 
		},
		2 * field_sett::size_field_unit);
	{
		DebugFieldMat mat;
		add_lines_img(mat, lines);
		debug("befor_rot", mat);
	}
	corners_rot(lines, -ang);
	{
		DebugFieldMat mat;
		add_lines_img(mat, lines);
		debug("Init_robot_from_start", mat);
	}

	std::array<std::pair<int, int>, 4> extra_line = { 
		std::make_pair(-1, 0),
		std::make_pair(-1, 0),
		std::make_pair(-1, 0),
		std::make_pair(-1, 0) 
	};
	for (int i = 0; i < lines.size(); i++) {
		for (int j = 0; j < (lines[i].size() - 1); j++) {
			if (lines[i][j].dist(lines[i][j + 1]) > (min_length)) {
				field_margin line_type;
				if (fabs(lines[i][j].get_x() - lines[i][j + 1].get_x()) >
				    fabs(lines[i][j].get_y() - lines[i][j + 1].get_y())) {
					if (lines[i][j].get_y() > 0) {
						line_type = top_field_margin;
					}
					else {
						line_type = bottom_field_margin;
					}
				}
				else {
					if (lines[i][j].get_x() > 0) {
						line_type = right_field_margin;
					}
					else {
						line_type = left_field_margin;
					}
				}
				if ((extra_line[line_type].first == -1) ||
				    dist_line2point(
				        lines[extra_line[line_type].first][extra_line[line_type].second],
					lines[extra_line[line_type].first][extra_line[line_type].second + 1],
					{ 0, 0 }) < dist_line2point(lines[i][j], lines[i][j + 1], { 0, 0 })) {
					extra_line[line_type].first = i;
					extra_line[line_type].second = j;
				}
			}
		}
	}
	RobotPoint pos;
	std::vector<std::pair<int, PolarPoint>> suspicious_points;
	for (int i = 0; i < extra_line.size(); i++) {
		if (extra_line[i].first >= 0) {
			suspicious_points.emplace_back(i,
				PolarPoint(dist_line2point(lines[extra_line[i].first][extra_line[i].second],
						lines[extra_line[i].first][extra_line[i].second + 1],
						{ 0, 0 }),
					get_middle_line_ang(lines[extra_line[i].first][extra_line[i].second],
						lines[extra_line[i].first][extra_line[i].second + 1],
						Point{ 0, 0 }) + ang));
		}
	}
	{
		DebugFieldMat mat;
		add_lines_img(mat, lines);
		for (int i = 0; i < suspicious_points.size(); i++)
		{
			PolarPoint p = suspicious_points[i].second;
			p.add_f(-ang);
			add_point_img(mat, p.to_cartesian());	
		}
		debug("Init_robot_from_start", mat);
	}	
	if (suspicious_points.size() > 1) {
		std::vector<std::pair<int, color_t>> colors = robot.GetColorFromAng(suspicious_points);
		for (int i = 0; i < colors.size(); i++) {
			if (colors[i].first != suspicious_points[i].first) {
				std::cerr << "Misha, BLIA. It might need sort!!!" << std::endl;
				continue;
			}
			if (colors[i].second == black_c)
			{
				RobotPoint p = dist2coordinates(
				    dist_line2point(lines[extra_line[colors[i].first].first][extra_line[colors[i].first].second],
						lines[extra_line[colors[i].first].first][extra_line[colors[i].first].second + 1],
						{ 0, 0 }),
					i);
				pos.merge(p);
			}
		}
	}
	pos.set_angle(ang);
	return pos;
}

void update_box_color(Robot &robot, Map &map) {
    auto boxes = map.get_boxes();
    std::vector<std::pair<int, PolarPoint>> points;
    for (int i = 0; i < boxes.size(); i++) {
        if(boxes[i].get_color() == undefined_c) {
            Point box = boxes[i].get_left_corner_point() + field_sett::climate_box_width / 2.;
            double ang = atan2(box.get_y() - map.get_position().get_y(),
                box.get_x() - map.get_position().get_x());
            points.emplace_back(i, PolarPoint{boxes[i].get_left_corner_point().dist(map.get_position()),
                                              ang});
        }
    }
    auto colors_box = robot.GetColorFromAng(points);
    for (int i = 0; i < colors_box.size(); i++) {
        if ((colors_box[i].second != black_c) && (colors_box[i].second != undefined_c) && (colors_box[i].second != white_c)) {
                map.set_box_color(colors_box[i].first, colors_box[i].second);
        } else {
            std::cout << "Undefined box " << colors_box[i].first << std::endl;
        }
    }
}

bool get_box(color_t color, Map &map, BoxMap &box) {
    auto boxes = map.get_boxes();
    for (int i = 0; i < boxes.size(); i++) {
        if (boxes[i].get_color() == color) {
            box = boxes[i];
            box.set_left_corner_point(Map::normal_point(box.get_left_corner_point()));
            return true;
        }
    }
}

void alg(Robot &robot) {
    show_img_debug debug;
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        debug = show_debug_img;
    #else
        debug = save_debug_img;
    #endif
    double frame_offset = out_way_offset;
    robot.Go2({{0, out_way_offset}});
    RobotPoint start_position;
    std::vector<PolarPoint> lidar_data;
    robot.GetLidarPolarPoints(lidar_data);
    start_position = detect_position(robot, lidar_data, frame_offset);
    while(!start_position.is_defined()) {
        robot.Go2({{0, 2 * field_sett::size_field_unit}});
        frame_offset += 2 * field_sett::size_field_unit;
        lidar_data.clear();
        start_position = detect_position(robot, lidar_data, frame_offset);
    }
    write_log("Position:/n x: " + std::to_string(start_position.get_x()) +
              " y: " + std::to_string(start_position.get_y()) +
              " ang: " + std::to_string(start_position.get_angle()));
    Point start_frame_point = Point{frame_offset * cos(start_position.get_angle()),
                                    frame_offset * sin(start_position.get_angle())} +
        start_position;
    Map map(start_position, Point{-field_sett::parking_zone_door_size * sin(start_position.get_angle()),
                                  -field_sett::parking_zone_door_size * cos(start_position.get_angle())} + start_frame_point,
            Point{field_sett::parking_zone_door_size * sin(start_position.get_angle()),
                  field_sett::parking_zone_door_size * cos(start_position.get_angle())} + start_frame_point);
    map.update(lidar_data);
    update_box_color(robot, map);
    color_t next_color = blue_c;
    std::vector<Point> way;
    Point end_move_point;
    Robot::CatchCubeSideEnum side_catch = Robot::CatchCubeSideEnum::LEFT;
    for (int u = 0; u < 3; u++) {
        BoxMap box;
        while (!get_box(next_color, map, box)) {
            Point death_point;
            auto death_zone = map.get_death_zone();
            for (int i = 0; i < death_zone.size(); i++) {
                for (int j = 0; j < death_zone[i].size(); j++) {
                    death_point = {i * field_sett::size_field_unit, j * field_sett::size_field_unit};
                    break;
                }
                if (!std::isnan(death_point.get_x())) {
                    break;
                }
            }
            go_to2(map, death_point, way, end_move_point, false, debug);
            robot.Go2(way);
            map.set_new_position(end_move_point);
            robot.GetLidarPolarPoints(lidar_data);
            map.update(lidar_data);
            update_box_color(robot, map);
        }
        next_color = do_box(robot, map, box, side_catch, true, false, debug);
    }
    if(!go_to2(map, start_position, way, end_move_point, false, debug))
        return;
    robot.Go2(way);
    frame_connect(robot, frame_offset, start_position.get_angle());
}