//
// Created by Danila on 18.06.2019.
//

#include "test.h"
#include "../highLogic/alg.h"
#include "../highLogic/CV.h"

TEST(TestDetectPosition, 1) {
    Robot robot;
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//8.ld", points));
    detect_position(robot, points, 300);
}

TEST(UpDateBox, 1) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(0);
    start_position.set_x(15 * field_sett::size_field_unit);
    start_position.set_y(11 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    Robot r;
    update_box_color(r, map);
}
