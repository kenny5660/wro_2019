//
// Created by Danila on 13.02.2019.
//

#ifndef LIDAR_MATH_TEST_H
#define LIDAR_MATH_TEST_H

#include <gtest/gtest.h>
#include "../lidar_math.h"

#define PRECISION_LENGTH (0.1)
#define PRECISION_ANGLE (0.001)

constexpr inline double degree2radian(double degree) {
    return degree * M_PI / 180;
}

#endif //LIDAR_MATH_TEST_H
