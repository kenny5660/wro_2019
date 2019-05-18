//
// Created by Danila on 06.05.2019.
//

#include "alg.h"
#include "map.h"
#include "CV.h"
#include "Move.h"
#include <opencv2/core.hpp>

Point catch_flower_offset[4] = {
    {-robot_sett::catch_flower_offset, 0},
    {0, robot_sett::catch_flower_offset},
    {robot_sett::catch_flower_offset, 0},
	{0, -robot_sett::catch_flower_offset}
};

Point catch_offset_driveway[4] = {
	{0, -robot_sett::catch_offset_driveway},
    {-robot_sett::catch_offset_driveway, 0},
    {0, robot_sett::catch_offset_driveway},
    {robot_sett::catch_offset_driveway, 0}
};

//TODO: анализ после мёртвой зоны

int turn2box(Robot &robot, BoxMap &box, Map &map) {
    int need_rot = 1;
    Point buff_p = box.get_box_indent() - Point{field_sett::size_field_unit, field_sett::size_field_unit};
    if (fabs(buff_p.get_x() - box.get_left_corner_point().get_x()) < fabs(buff_p.get_y() - box.get_left_corner_point().get_y())) {
        if (box.get_left_corner_point().get_y() > buff_p.get_y()) {
            need_rot = 0;
        } else {
            need_rot = 2;
        }
    } else {
        if (box.get_left_corner_point().get_x() > buff_p.get_x()) {
            need_rot = 3;
        }
    }
    double need_rot_rad = (need_rot) * M_PI_2 - map.get_position().get_angle();
    {
        write_log("Need rood: \n" + std::to_string(need_rot_rad));
    }
    robot.Turn(need_rot_rad);
    map.set_new_position(RobotPoint{map.get_position().get_x(), map.get_position().get_y(),
                                    -need_rot_rad +  map.get_position().get_angle()});
    return need_rot;
}

void do_alg_code(Robot &robot, bool kamikaze_mode) {
    const double out_way_offset = 300;

    clear_logs();
    Robot::CatchCubeSideEnum side_catch = Robot::CatchCubeSideEnum::LEFT;
    cv::Mat QRCodeImg;
    robot.GetQRCode(QRCodeImg);
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz);
    double start_angle = start_position.get_angle();
    {
        write_log("Start angle: " + std::to_string(start_angle));
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
    for (auto i : boxes) {
        std::vector<Point> way;
        int need_rot = turn2box(robot, i, map);
        {
            write_log("Found way to: \n"
                      " x = " + std::to_string(i.get_box_indent().get_x()) +
                      "\n y = " + std::to_string(i.get_box_indent().get_y()));
        }
	    Point end_point;
	    if (!go_to(map, i.get_box_indent(), way, end_point, kamikaze_mode)) {
            // удаляем последние 2 точки т. к. робот не двумерная шкура и подъезжать в упор опансо
            Point robot_p(0, 0);
            do {
                way.pop_back();
                way.pop_back();
                {
                    write_log("Way in death zone. \n"
                              "Go2: \n"
                              " x = " + std::to_string(way.back().get_x()) +
                        "\n y = " + std::to_string(way.back().get_y()));
                }
                robot.Go2(way);
	            map.set_new_position(RobotPoint{ end_point.get_x(), end_point.get_y(), map.get_position().get_angle() });
                std::vector<PolarPoint> lidar_data;
                robot.GetLidarPolarPoints(lidar_data);
                Map new_map(lidar_data);
                RobotPoint new_position = new_map.get_position();
                if (!(new_position.is_defined() && map.merge(new_map))) {
                    write_log("!!!Pizdec, iata chast' echo ne napisanna!!!");
                    return;
                    // TODO: Чтобы в стену не уебнуться
                }
		    } while (!go_to(map, i.get_box_indent(), way, end_point) && (way.size() > 1));
            if (way.size() < 2) {
                write_log("!!!Pizdec, iata chast' echo ne napisanna!!!");
                return;
                // TODO: Чтобы в стену не уебнуться
            }
            robot.Go2(way);
	        map.set_new_position(RobotPoint{ end_point.get_x(), end_point.get_y(), map.get_position().get_angle() });
        } else {
            {
                write_log("Way founded.");
            }
            robot.Go2(way);
	        map.set_new_position(RobotPoint{ end_point.get_x(), end_point.get_y(), map.get_position().get_angle() });
        }
        robot.CatchCube(side_catch);
        Point offset_catch = map.get_position() + catch_offset_driveway[need_rot] + catch_flower_offset[need_rot] * ((side_catch == Robot::CatchCubeSideEnum::LEFT) ? (1) : (-1));
        map.set_new_position(RobotPoint{offset_catch.get_x(), offset_catch.get_y(), map.get_position().get_angle()});
        side_catch = (side_catch == Robot::CatchCubeSideEnum::LEFT) ?
                     (Robot::CatchCubeSideEnum::RIGHT) :
                     (Robot::CatchCubeSideEnum::LEFT);
        {
            write_log("Position after catch: \n"
                      " x = " + std::to_string(way.back().get_x()) +
                      "\n y = " + std::to_string(way.back().get_y()));
        }
    }
    // TODO: вернуться домой
}