//
// Created by Danila on 29.01.2019.
//

#ifndef LIDAR_MATH_LIDAR_MATH_H
#define LIDAR_MATH_LIDAR_MATH_H

#include <math.h>
#include "logic_structures.h"
#include "settings.h"

RobotPoint init_position_from_line(double b, double c, double alpha);
RobotPoint init_position_from_corner(double a, double b, double c,
                                     double corner_ab, double corner_bc,
                                     double a_angle_offset);

//---FOR TEST---//
RobotPoint get_coordinates_from_line(double b, double c, double alpha,
                                     double b_angle_offset,
                                     field_margin margin);
RobotPoint get_coordinates_from_corner(double a, double b, double c,
                                       double corner_ab, double corner_bc,
                                       double a_angle_offset,
                                       field_corner corner);
//////

void location_recognition_model_test();

#endif //LIDAR_MATH_LIDAR_MATH_H
