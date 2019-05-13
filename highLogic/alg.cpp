//
// Created by Danila on 06.05.2019.
//

#include "alg.h"
#include "map.h"
#include "CV.h"
#include "Move.h"
#include <opencv2/core.hpp>

Point catch_flower_offset[4] = {
    {0, -robot_sett::catch_flower_offset},
    {-robot_sett::catch_flower_offset, 0},
    {0, robot_sett::catch_flower_offset},
    {robot_sett::catch_flower_offset, 0}
};

void do_alg_code(Robot &robot) {
    Robot::CatchCubeSideEnum side_catch = Robot::CatchCubeSideEnum::LEFT;
    cv::Mat QRCodeImg;
    robot.GetQRCode(QRCodeImg);
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz);
    double start_angle = start_position.get_angle();
    robot.Turn(start_angle);
    start_position.set_angle(0);
    Map map(pz.first, pz.second, boxes, start_position);
    start_position.set_angle(start_angle);
    save_debug_img("QRCodeDetection", map.get_img());
    for (auto i : boxes) {
        std::vector<Point> way;
        if(!go_to(map, i.get_box_indent(), way)) { // уточнить нужно ли отрицание

            // удаляем последние 2 точки т. к. робот не двумерная шкура и подъезжать в упор опансо
            way.pop_back();
            way.pop_back();
            robot.Go2(way);
            std::vector<PolarPoint> lidar_data;
            robot.GetLidarPolarPoints(lidar_data);
            Map new_map(lidar_data);
            RobotPoint new_position = new_map.get_position();
            if (!(new_position.is_defined() && map.merge(new_map))) {
                // TODO: Чтобы в стену не уебнуться
            }
        } else {
            robot.Go2(way);
        }
        int need_rot = 0;
        Point buff_p = i.get_box_indent() - Point{field_sett::size_field_unit, field_sett::size_field_unit};
        if (fabs(buff_p.get_x() - i.get_left_corner_point().get_x()) < fabs(buff_p.get_y() - i.get_left_corner_point().get_y())) {
            if (i.get_left_corner_point().get_y() > buff_p.get_y()) {
                need_rot = 3;
            } else {
                need_rot = 1;
            }
        } else {
            if (i.get_left_corner_point().get_x() > buff_p.get_x()) {
                need_rot = 2;
            }
        }
        double need_rot_rad = need_rot * M_PI_2 - map.get_position().get_angle();
        robot.Turn(need_rot_rad);
        robot.CatchCube(side_catch);
        Point offset_catch = map.get_position() + catch_flower_offset[need_rot] * ((side_catch == Robot::CatchCubeSideEnum::LEFT) ? (1) : (-1));
        map.set_new_position(RobotPoint{offset_catch.get_x(), offset_catch.get_y(), need_rot_rad});
        side_catch = (side_catch == Robot::CatchCubeSideEnum::LEFT) ?
                     (Robot::CatchCubeSideEnum::RIGHT) :
                     (Robot::CatchCubeSideEnum::LEFT);
    }
    // TODO: вернуться домой
}