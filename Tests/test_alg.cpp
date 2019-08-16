//
// Created by Danila on 18.06.2019.
//

#include "test.h"
#include "../highLogic/alg.h"
#include "../highLogic/CV.h"

extern RobotPoint detect_position(Robot &robot, std::vector<PolarPoint> &lidar_data, double frame_offset);

TEST(TestDetectPosition, 1) {
    Robot robot;
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//8.ld", points));
    detect_position(robot, points, 300);
}

TEST(TestDetectPosition, 2) {
    Robot robot;
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//10.ld", points));
    detect_position(robot, points, 300);
}

TEST(TestDetectPosition, 3) {
    Robot robot;
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("12_15_42_2019_06_16.ld", points));
    detect_position(robot, points, 300);
}

TEST(UpDateBox, 1) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(boxes, pz, "(O,K,M,M)(G,G,E,I)(F,N,D,P)(J,R,H,T)");
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
}


