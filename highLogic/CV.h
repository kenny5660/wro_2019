//
// Created by Danila on 06.05.2019.
//

#ifndef LIDAR_MATH_CV_H
#define LIDAR_MATH_CV_H

#include "logic_structures.h"
#include "map.h"
#include <queue>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

RobotPoint qr_detect(std::array<BoxMap, 3> &boxes_pos, std::pair<Point, Point> &pz, std::string s = "");

#endif //LIDAR_MATH_CV_H
