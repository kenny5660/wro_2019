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
enum class QrDetectorTypeEnum
{
	CV,
	ZBAR
}
;
const QrDetectorTypeEnum kQrDetectorType = QrDetectorTypeEnum::CV;//QrDetectorTypeEnum::ZBAR;
std::string qr_detect_frame(cv::Mat qr);
RobotPoint qr_detect(cv::Mat qr, std::array<BoxMap, 3> &boxes_pos, std::pair<Point, Point> &pz, std::string s = "");

#endif //LIDAR_MATH_CV_H
