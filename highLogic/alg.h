//
// Created by Danila on 06.05.2019.
//

#ifndef LIDAR_MATH_ALG_H
#define LIDAR_MATH_ALG_H

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    #include "../wro2019main/Robot_low_Lvl_lib/Robot_facing.h"
#else
    #include "../wro2019main/Robot_low_Lvl_lib/Robot.h"
#endif
#include "debug.h"
void do_alg_code(Robot &robot);
#endif //LIDAR_MATH_ALG_H
