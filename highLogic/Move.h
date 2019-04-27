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

bool go_to(const Map &map, const Point &point, std::vector<Point> &ans);

#endif //LIDAR_MATH_MOVE_H
