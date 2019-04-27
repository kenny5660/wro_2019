//
// Created by Danila on 13.02.2019.
//

#ifndef LIDAR_MATH_TEST_H
#define LIDAR_MATH_TEST_H

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../highLogic/lidar_math.h"
#include "../highLogic/debug.h"
#include "../highLogic/Map.h"
#include "../highLogic/Move.h"

#define PRECISION_LENGTH (0.1)
#define PRECISION_ANGLE (0.001)

#define PRECISION_LENGTH_POS (20.0)
#define PRECISION_ANGLE_POS (0.05)

#define PRECISION_REAL_LENGTH ((PRECISION_LENGTH + field_sett::truncation_field_error) * 2)

const std::string sources_path = "../Tests/source_test/";
const std::string img_path = sources_path + "img/";
const std::string lidar_data_path = sources_path + "lidar_data/";

const std::string lidar_emulator_path = "../Tests/lidar_math/";
const std::string lidar_emulator_name = "WRO_2019_LIDAR_TEST.jar";
const std::string lidar_stream_name = "data.ld";

constexpr inline double degree2radian(double degree) {
    return degree * M_PI / 180;
}

bool read(std::string s, std::vector<PolarPoint> &points, std::string path = lidar_data_path);

std::string lines2string(const std::vector<std::vector<Point>> &p);

#endif //LIDAR_MATH_TEST_H
