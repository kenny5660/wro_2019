//
// Created by Danila on 06.05.2019.
//

#ifndef LIDAR_MATH_ALG_H
#define LIDAR_MATH_ALG_H

#include "debug.h"
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    #include "../wro2019main/Robot_low_Lvl_lib/Robot_facing.h"
#else
    #include "../wro2019main/Robot_low_Lvl_lib/Robot.h"
#endif

const double out_way_offset = 300;

void frame_connect(Robot &robot, double out_way_offset, double start_angle);
RobotPoint detect_position(Robot &robot, std::vector<PolarPoint> &lidar_data, double frame_offset);

void do_alg_code(Robot &robot, bool kamikaze_mode = false, std::string s = "");
void alg(Robot &robot);
#endif //LIDAR_MATH_ALG_H
