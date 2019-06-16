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

void catch_box(Robot &robot, Robot::CatchCubeSideEnum side_catch, Point catch_flower_off) {
    //
	//box_connect(robot);
    robot.CatchCube(side_catch);
}

void frame_connect(Robot &robot, double out_way_offset, double start_angle) {
    Point point_offset = {out_way_offset, -field_sett::parking_zone_door_size / 2.};
    std::vector<PolarPoint> lidar_dt;
    robot.GetLidarPolarPoints(lidar_dt);
    auto points = get_corners(lidar_dt);
    std::pair<int, int> ind_nearly_point = {0, 0};
    for (int i = 1; i < points.size(); i++) {
        for (int j = 0; j < points[i].size(); j++) {
            if (points[ind_nearly_point.first][ind_nearly_point.second].dist(point_offset) >
                points[i][j].dist(point_offset)) {
                ind_nearly_point = std::make_pair(i, j);
            }
        }
    }
    robot.Go2({{points[ind_nearly_point.first][ind_nearly_point.second].get_y(), -points[ind_nearly_point.first][ind_nearly_point.second].get_x() - field_sett::parking_zone_door_size / 2.0}});
    robot.Turn(-start_angle);
}

Point go_from_frame(Robot &robot, double dist, double ang) {
    robot.Go2({{0, dist}});
	robot.Turn(ang);
    return {cos(ang) * dist, sin(ang) * dist};
}

void do_alg_code(Robot &robot, bool kamikaze_mode, std::string s) {
    const double out_way_offset = 300;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	clear_logs();
#endif

    Robot::CatchCubeSideEnum side_catch = Robot::CatchCubeSideEnum::LEFT;
    cv::Mat QRCodeImg;
    if (s == "") {
        robot.GetQRCode(QRCodeImg);
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
        way.clear();
        int need_rot = turn2box(robot, i, map);
        {
            write_log("Found way to: \n"
                      " x = " + std::to_string(i.get_box_indent().get_x()) +
                      "\n y = " + std::to_string(i.get_box_indent().get_y()));
        }
	    Point end_point;
        bool way_found = go_to2(map, i.get_box_indent(), way, end_point, kamikaze_mode, db);
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
	        {
		        save_debug_img("map_after", map.get_img());
	        }
            std::vector<PolarPoint> ld;
            robot.GetLidarPolarPoints(ld);
            robot.Turn(ang);
            pos.add_angle(-ang);
            map.set_new_position(pos);
            way_found = go_to2(map, i.get_box_indent(), way, end_point, kamikaze_mode, db);
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
        catch_box(robot, side_catch, catch_flower_offset[need_rot]);
	    side_catch = (side_catch == Robot::CatchCubeSideEnum::LEFT) ?
                     (Robot::CatchCubeSideEnum::RIGHT) :
                     (Robot::CatchCubeSideEnum::LEFT);
        {
            write_log("Position after catch: \n"
                      " x = " + std::to_string(way.back().get_x()) +
                      "\n y = " + std::to_string(way.back().get_y()));
        }
    }
    way.clear();
    Point b;
    if(!go_to2(map, start_position, way, b, kamikaze_mode, db))
        return;
    robot.Turn(-map.get_position().get_angle() + M_PI_2);
    robot.Go2(way);
    frame_connect(robot, out_way_offset, start_angle);
    //
}