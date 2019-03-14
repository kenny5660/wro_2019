//
// Created by Danila on 13.02.2019.
//

#include "../test.h"

TEST(PositionFromDownLine, PerpendicularLeftTriangle) {
    RobotPoint p = get_coordinates_from_line(13.217, 9.378, 0.68783426,
        3.24735961, bottom_field_margin);
    EXPECT_TRUE(std::isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), (MAX_FIELD_HEIGHT - 9.36), PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), 0.67806041, PRECISION_ANGLE);
}

TEST(PositionFromDownLine, PerpendicularInTriangle) {
    RobotPoint p = get_coordinates_from_line(13.145, 12.526, 0.8841838,
        2.41501209, bottom_field_margin);
    EXPECT_TRUE(std::isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), (MAX_FIELD_HEIGHT - 11.58), PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), 1.804386, PRECISION_ANGLE);
}

TEST(PositionFromDownLine, PerpendicularRightTriangle) {
    RobotPoint p = get_coordinates_from_line(21.933, 30.99, 0.38117991,
        3.29378536, bottom_field_margin);
    EXPECT_TRUE(std::isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), (MAX_FIELD_HEIGHT - 18.867), PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), 1.95372156, PRECISION_ANGLE);
}

TEST(PositionFromDownLine, PerpendicularIsCParametr) {
    RobotPoint p = get_coordinates_from_line(28.765, 20.270, 0.7888888,
        -1.1718141, bottom_field_margin);
    EXPECT_TRUE(std::isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), (MAX_FIELD_HEIGHT - 20.27), PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), 5.09531422, PRECISION_ANGLE);
}

TEST(PositionFromRightLine, PerpendicularIsDown) {
    RobotPoint p = get_coordinates_from_line(12.119, 10.460,
        degree2radian(18.91), degree2radian(35.83), right_field_margin);
    EXPECT_NEAR(p.get_x(), (MAX_FIELD_WIDTH - 10.134), PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), degree2radian(110.90), PRECISION_ANGLE);
}

TEST(PositionFromRightLine, PerpendicularIsUp) {
    RobotPoint p = get_coordinates_from_line(10.801, 13.582,
        degree2radian(30.47), degree2radian(187.48), right_field_margin);
    EXPECT_NEAR(p.get_x(), (MAX_FIELD_WIDTH - 10.709), PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), degree2radian(0), PRECISION_ANGLE);
}

TEST(PositionFromRightLine, PerpendicularIsInDownZero) {
    RobotPoint p = get_coordinates_from_line(9.180, 8.803,
        degree2radian(43.63), degree2radian(-63.87), right_field_margin);
    EXPECT_NEAR(p.get_x(), (MAX_FIELD_WIDTH - 8.332), PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), degree2radian(180 + 39.06), PRECISION_ANGLE);
}

TEST(PositionFromRightLine, PerpendicularIsInUpZero) {
    RobotPoint p = get_coordinates_from_line(7.538, 7.250,
        degree2radian(40.48), degree2radian(21.82), right_field_margin);
    EXPECT_NEAR(p.get_x(), (MAX_FIELD_WIDTH - 6.925), PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), degree2radian(180 - 45.09), PRECISION_ANGLE);
}

TEST(PositionFromTopLine, PerpendicularIsLeft) {
    RobotPoint p = get_coordinates_from_line(13.360, 17.590,
        degree2radian(15.21), degree2radian(-166.81), top_field_margin);
    EXPECT_TRUE(std::isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), 10.519, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(270 + 24.87), PRECISION_ANGLE);
}

TEST(PositionFromTopLine, PerpendicularIsRight) {
    RobotPoint p = get_coordinates_from_line(6.417, 5.505,
        degree2radian(16.16), degree2radian(30.95), top_field_margin);
    EXPECT_TRUE(std::isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), 5.165, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(22.65), PRECISION_ANGLE);
}

TEST(PositionFromTopLine, PerpendicularIsIn) {
    RobotPoint p = get_coordinates_from_line(7.852, 9.332,
        degree2radian(74.76), degree2radian(16.60), top_field_margin);
    EXPECT_TRUE(std::isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), 6.734, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(90 - 47.54), PRECISION_ANGLE);
}

TEST(PositionFromLeftLine, PerpendicularIsDown) {
    RobotPoint p = get_coordinates_from_line(9.881, 11.7,
        degree2radian(17.17), degree2radian(83.15), left_field_margin);
    EXPECT_NEAR(p.get_x(), 9.25, PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), degree2radian(270 + 27.44), PRECISION_ANGLE);
}

TEST(PositionFromLeftLine, PerpendicularIsUp) {
    RobotPoint p = get_coordinates_from_line(13.874, 12.709,
        degree2radian(17.69), degree2radian(-74.33), left_field_margin);
    EXPECT_NEAR(p.get_x(), 12.617, PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), degree2radian(90 - 40.25), PRECISION_ANGLE);
}

TEST(PositionFromLeftLine, PerpendicularIsIn) {
    RobotPoint p = get_coordinates_from_line(10.993, 11.381,
        degree2radian(28.46), degree2radian(-153.25), left_field_margin);
    EXPECT_NEAR(p.get_x(), 10.815, PRECISION_LENGTH);
    EXPECT_TRUE(std::isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), degree2radian(180 - 37.07), PRECISION_ANGLE);
}

TEST(PositionFromCorner, TopLeftCorner) {
    RobotPoint p = get_coordinates_from_corner(6.93, 8.935, 6.35,
        degree2radian(24.13), degree2radian(29.78), degree2radian(-20),
        top_left_corner);
    EXPECT_NEAR(p.get_x(), 6.571, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), 6.054, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(38.53), PRECISION_ANGLE);
}

TEST(PositionFromCorner, TopRightCorner) {
    // a и c есть перпендикуляры
    RobotPoint p = get_coordinates_from_corner(1.712, 10.607, 10.468,
        degree2radian(80.71), degree2radian(9.29),
        degree2radian(68.51), top_right_corner);
    EXPECT_NEAR(p.get_x(), MAX_FIELD_WIDTH - 10.468, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), 1.712, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(90 - 68.51), PRECISION_ANGLE);
}

TEST(PositionFromCorner, BottomRightCorner) {
    RobotPoint p = get_coordinates_from_corner(10.772, 12.022, 7.578,
        degree2radian(65.23), degree2radian(37.51), degree2radian(147.23),
        bottom_right_corner);
    EXPECT_NEAR(p.get_x(), MAX_FIELD_WIDTH - 9.536, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), MAX_FIELD_HEIGHT - 7.32, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(5.06), PRECISION_ANGLE);
}

TEST(PositionFromCorner, BottomLeftCorner) {
    RobotPoint p = get_coordinates_from_corner(11.847, 10.445, 9.227,
                   degree2radian(90), degree2radian(90), degree2radian(41.43),
                   bottom_left_corner);
    EXPECT_NEAR(p.get_x(), 6.918, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), MAX_FIELD_HEIGHT - 7.839, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(180), PRECISION_ANGLE);
}

TEST(PositionFromCorner, BottomLeftCornerFreeField) {
    RobotPoint p = get_coordinates_from_corner(13.817, 7.551, 10.423,
        degree2radian(112.66), degree2radian(103.75), degree2radian(-67.38),
        bottom_left_corner);
    EXPECT_NEAR(p.get_x(), 5.365, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), MAX_FIELD_HEIGHT - 5.314, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), degree2radian(270), PRECISION_ANGLE);
}
