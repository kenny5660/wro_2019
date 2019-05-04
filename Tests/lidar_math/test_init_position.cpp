//
// Created by Danila on 13.02.2019.
//

#include "../test.h"

TEST(InitPositionFromLine, LookUp) {
    RobotPoint p = init_position_from_line(11.181, 8.151, 0.523599);
    EXPECT_NEAR(p.get_x(), 7.861, PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), 5.75382695, PRECISION_ANGLE);
}

TEST(InitPositionFromLine, LookDown) {
    RobotPoint p = init_position_from_line(17.033, 38.489, 0.349066);
    EXPECT_NEAR(p.get_x(), 9.654, PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), 1.1426671, PRECISION_ANGLE);
}

TEST(InitPositionFromLine, LookPerpendicular) {
    RobotPoint p = init_position_from_line(44.487, 44.487, 0.244346);
    EXPECT_NEAR(p.get_x(), 44.155, PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_DOUBLE_EQ(p.get_angle(), 0);
}

TEST(InitPositionFromCorner, ZeroLineDown) {
    RobotPoint p = init_position_from_corner(7.595, 11.268, 10.190,
        degree2radian(32.05), degree2radian(7.95), degree2radian(-20));
    EXPECT_NEAR(p.get_x(), 7.219, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), 8.652, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(38.11), PRECISION_ANGLE);
}

TEST(InitPositionFromCorner, ZeroLineUp) {
    RobotPoint p = init_position_from_corner(6.934, 9.579, 7.675,
        degree2radian(19.53), degree2radian(30.47), degree2radian(-25));
    EXPECT_NEAR(p.get_x(), 5.803, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), 7.621, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(58.18), PRECISION_ANGLE);
}

TEST(RealInit, Line) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Init_from_line.ld", points));
    double dist;
    auto p = init_pos(points, dist);
    EXPECT_NEAR(p.first.get_x(), 2144, PRECISION_LENGTH_POS);
    EXPECT_TRUE(std::isnan(p.first.get_y()));
    EXPECT_NEAR(p.first.get_angle(), degree2radian(17.4), PRECISION_ANGLE_POS);
}

TEST(RealInit, LineDown) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Init_from_line_down.ld", points));
    double dist;
    auto p = init_pos(points, dist);
    EXPECT_NEAR(p.first.get_x(), 1316, PRECISION_LENGTH_POS);
    EXPECT_TRUE(std::isnan(p.first.get_y()));
    EXPECT_NEAR(p.first.get_angle(), degree2radian(320), PRECISION_ANGLE_POS);
}

TEST(RealInit, Corner) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Init_from_corner.ld", points));
    double dist;
    auto p = init_pos(points, dist);
    EXPECT_NEAR(p.first.get_x(), 1048, PRECISION_LENGTH_POS);
    EXPECT_NEAR(p.first.get_y(), 952, PRECISION_LENGTH_POS);
    EXPECT_NEAR(p.first.get_angle(), degree2radian(42), PRECISION_ANGLE_POS);
}