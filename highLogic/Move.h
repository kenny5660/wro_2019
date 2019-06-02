//
// Created by Danila on 19.04.2019.
//

#ifndef LIDAR_MATH_MOVE_H
#define LIDAR_MATH_MOVE_H

#include "map.h"
#include "logic_structures.h"

const int width_mesh = 20; // должно делится на 20 (И это логично!)
const int height_mesh = 20;

const int unit_offset = 1;

bool go_to(Map &map, const Point &point, std::vector<Point> &ans, Point &end_point, bool kamikaze_mode = false, show_img_debug debug = nullptr);

bool go_to2(Map &map, const Point &point, std::vector<Point> &ans, Point &end_point, bool kamikaze_mode = false, show_img_debug debug = nullptr);

#endif //LIDAR_MATH_MOVE_H
