//
// Created by Danila on 20.04.2019.
//

#include "test.h"
#include "../highLogic/CV.h"
#include "../highLogic/alg.h"

TEST(Move, MakeWayFree) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot3/0.ld", points));
    Map m1(points);
    std::vector<Point> way;
    Point end_p;
    EXPECT_TRUE(go_to(m1, {field_sett::max_field_width - 120, 120}, way, end_p));
    EXPECT_FALSE(way.empty());
}

TEST(Move, MakeWayDeath) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot3/0.ld", points));
    Map m1(points);
    show_debug_img("P", m1.get_img());
    std::vector<Point> way;
    Point end_p;
    EXPECT_FALSE(go_to(m1, {1000, 500}, way, end_p));
    EXPECT_FALSE(way.empty());
}

TEST(Move, addPZ) {
    //зайти ручками и проверить, что на нужном месте позиция занята
    Map m1({480, 120}, {480, 120 + field_sett::parking_zone_door_size});
    std::vector<Point> way;
    Point end_p;
    go_to(m1, {1000, 500}, way, end_p);
}

TEST(Move2, NullCrossKamicadze) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(H,K,K,K)(A,A,C,C)(I,A,K,C)(E,A,G,C)");
    start_position.set_angle(0);
    start_position.set_x(3 * field_sett::size_field_unit);
    start_position.set_y(12 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<Point> ans;
    Point end_p;
    go_to2(map, Point{4 * field_sett::size_field_unit, 8 * field_sett::size_field_unit}, ans, end_p, true, show_debug_img);
}

TEST(Move2, NullCross) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(H,K,K,K)(A,A,C,C)(I,A,K,C)(E,A,G,C)");
    start_position.set_angle(0);
    start_position.set_x(3 * field_sett::size_field_unit);
    start_position.set_y(12 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<Point> ans;
    Point end_p;
    go_to2(map, Point{4 * field_sett::size_field_unit, 8 * field_sett::size_field_unit}, ans, end_p,
           false, show_debug_img);
}

TEST(Move2, Cross2Corner) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(H,K,K,K)(A,A,C,C)(I,A,K,C)(E,A,G,C)");
    start_position.set_angle(0);
    start_position.set_x(3 * field_sett::size_field_unit);
    start_position.set_y(12 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<Point> ans;
    Point end_p;
    go_to2(map, Point{16 * field_sett::size_field_unit, 12 * field_sett::size_field_unit}, ans, end_p, false, show_debug_img);
}

TEST(Move2, Cross1CornerKamicadze) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(H,K,K,K)(A,A,C,C)(I,A,K,C)(E,A,G,C)");
    start_position.set_angle(0);
    start_position.set_x(3 * field_sett::size_field_unit);
    start_position.set_y(12 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<Point> ans;
    Point end_p;
    go_to2(map, Point{8 * field_sett::size_field_unit, 8 * field_sett::size_field_unit}, ans, end_p,
           true, show_debug_img);
}

TEST(Move2, Cross2) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(N,G,O,I)(Q,Q,O,S)(F,G,D,I)(U,L,S,N)");
    start_position.set_angle(0);
    start_position.set_x(4 * field_sett::size_field_unit);
    start_position.set_y(8 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<Point> ans;
    Point end_p;
    go_to2(map, Point{12 * field_sett::size_field_unit, 4 * field_sett::size_field_unit}, ans, end_p,
           false, show_debug_img);
}

TEST(Move2, Cross2KamicadtheCross) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(N,G,O,I)(Q,Q,O,S)(F,G,D,I)(U,L,S,N)");
    start_position.set_angle(0);
    start_position.set_x(4 * field_sett::size_field_unit);
    start_position.set_y(8 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<Point> ans;
    Point end_p;
    go_to2(map, Point{16 * field_sett::size_field_unit, 16 * field_sett::size_field_unit}, ans, end_p,
           false, show_debug_img);
}

TEST(Move2, Cross2KamicadtheCross2) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(N,G,O,I)(Q,Q,O,S)(F,G,D,I)(U,L,S,N)");
    start_position.set_angle(0);
    start_position.set_x(4 * field_sett::size_field_unit);
    start_position.set_y(8 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<Point> ans;
    Point end_p;
    go_to2(map, Point{12 * field_sett::size_field_unit, 8 * field_sett::size_field_unit}, ans, end_p,
           false, show_debug_img);
}

TEST(Move2, RealKamicadthe) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(0);
    start_position.set_x(11 * field_sett::size_field_unit);
    start_position.set_y(7 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<Point> ans;
    Point end_p;
    go_to2(map, Point{8 * field_sett::size_field_unit, 14 * field_sett::size_field_unit}, ans, end_p,
           true, show_debug_img);
}

TEST(Move2, RealKamicadthe2) {
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
    std::vector<Point> ans;
    Point end_p;
    go_to2(map, Point{383.3, 1208.4}, ans, end_p,
           true, show_debug_img);
}