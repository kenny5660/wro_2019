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

    Point center = position_box_side(lidar_data, 1, 1, save_debug_img);
    robot.Go2({{-(field_sett::climate_box_offset / 2 - center.get_y()), 0}});
    robot.Go2({{0, -center.get_x()}});
}

void catch_box(Robot &robot, Robot::CatchCubeSideEnum side_catch, Point catch_flower_off) {
    box_connect(robot);
    robot.CatchCube(side_catch);
    robot.Go2({{-robot_sett::catch_offset_driveway, 0}});
	robot.Go2({catch_flower_off * ((side_catch == Robot::CatchCubeSideEnum::LEFT) ? (1) : (-1))});
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
    robot.Turn(start_angle);
    robot.Go2({Point{0, out_way_offset}});
    start_position.set_angle(0);
    start_position.set_x(start_position.get_x() - out_way_offset);
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
    for (auto i : boxes) {
        way.clear();
        int need_rot = turn2box(robot, i, map);
        {
            write_log("Found way to: \n"
                      " x = " + std::to_string(i.get_box_indent().get_x()) +
                      "\n y = " + std::to_string(i.get_box_indent().get_y()));
        }
	    Point end_point;
        show_img_debug db;
        #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
            db = show_debug_img;
        #else
            db = save_debug_img;
        #endif
        bool way_found = go_to2(map, i.get_box_indent(), way, end_point, kamikaze_mode, db);
        robot.Go2(way);
        map.set_new_position(RobotPoint{ end_point.get_x(), end_point.get_y(), map.get_position().get_angle() });
        while (!way_found) {
            double ang = M_PI - atan2(
                way[way.size() - 1].get_y() - way[way.size() - 2].get_y(),
                way[way.size() - 1].get_y() - way[way.size() - 2].get_y());
            ang = PolarPoint::angle_norm(ang) - map.get_position().get_angle();
            robot.Turn(ang);
            RobotPoint pos = map.get_position();
            pos.add_angle(ang);
            map.set_new_position(pos);
            std::vector<PolarPoint> ld;
            robot.GetLidarPolarPoints(ld);
            robot.Turn(-ang);
            pos.add_angle(-ang);
            map.set_new_position(pos);
            way_found = go_to2(map, i.get_box_indent(), way, end_point, kamikaze_mode, db);
            if (!way_found && (way.size() == 2) &&
               ((fabs(way.front().get_x() - way.back().get_x())) < 10) &&
               ((fabs(way.front().get_x() - way.back().get_x())) < 10)) {
                //TODO: если нет путит
            } else {
                robot.Go2(way);
                map.set_new_position(RobotPoint{ end_point.get_x(), end_point.get_y(), map.get_position().get_angle() });
            }
        }
            {
                write_log("Way founded.");
            }
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
    go_to2(map, Point{start_position.get_x() - out_way_offset, start_position.get_y()}, way, b, kamikaze_mode);
    // TODO: вернуться домой
}