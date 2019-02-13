//
// Created by Danila on 29.01.2019.
//

#ifndef LIDAR_MATH_LIDAR_MATH_H
#define LIDAR_MATH_LIDAR_MATH_H

#include <math.h>
#include "logic_structures.h"
#include "settings.h"

RobotPoint init_position_from_line(double b, double c, double alpha);
RobotPoint get_coordinates_from_line(double b, double c, double alpha,
                                     double b_angle_offset,
                                     field_margin margin);

void location_recognition_model_test();

#endif //LIDAR_MATH_LIDAR_MATH_H
