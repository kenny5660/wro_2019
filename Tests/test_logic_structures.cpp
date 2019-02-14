//
// Created by Danila on 14.02.2019.
//

#include "test.h"

TEST(PointClass, Init) {
    Point point(5);
    EXPECT_DOUBLE_EQ(point.get_x(), 5);
    EXPECT_TRUE(isnan(point.get_y()));
}

TEST(PolarPointClass, Init) {
    PolarPoint point(5);
    EXPECT_DOUBLE_EQ(point.get_r(), 5);
    EXPECT_TRUE(isnan(point.get_f()));
}

TEST(PolarPointClass, Angle) {
    PolarPoint point(0, degree2radian(30));
    EXPECT_NEAR(point.get_f(), degree2radian(30), PRECISION_ANGLE);
    point.add_f(degree2radian(365));
    EXPECT_NEAR(point.get_f(), degree2radian(35), PRECISION_ANGLE);
    point.add_f(degree2radian(-40));
    EXPECT_NEAR(point.get_f(), degree2radian(355), PRECISION_ANGLE);
    point.set_f(degree2radian(720));
    EXPECT_DOUBLE_EQ(point.get_f(), 0);
    point.set_f(degree2radian(-725));
    EXPECT_NEAR(point.get_f(), degree2radian(355), PRECISION_ANGLE);
}

TEST(PolarPointClass, ToCartesian) {
    PolarPoint polar_point(15.25, degree2radian(16));
    Point point = polar_point.to_cartesian();
    EXPECT_NEAR(point.get_x(), 14.65924, PRECISION_LENGTH);
    EXPECT_NEAR(point.get_y(), 4.203469, PRECISION_LENGTH);
    polar_point.set_f(degree2radian(355));
    point = polar_point.to_cartesian();
    EXPECT_NEAR(point.get_x(), 15.19197, PRECISION_LENGTH);
    EXPECT_NEAR(point.get_y(), -1.32913, PRECISION_LENGTH);
}

TEST(RobotPointClass, Init) {
    RobotPoint point(0);
    EXPECT_DOUBLE_EQ(point.get_x(), 0);
    EXPECT_TRUE(isnan(point.get_y()));
    EXPECT_TRUE(isnan(point.get_angle()));
}

TEST(RobotPointClass, Angle) {
    RobotPoint point(0, 0, degree2radian(530));
    EXPECT_NEAR(point.get_angle(), degree2radian(170), PRECISION_ANGLE);
    point.set_angle(degree2radian(-10));
    EXPECT_NEAR(point.get_angle(), degree2radian(350), PRECISION_ANGLE);
    point.add_angle(degree2radian(370));
    EXPECT_NEAR(point.get_angle(), 0, PRECISION_ANGLE);
}

TEST(RobotPointClass, Merge) {
    RobotPoint point(3, 2);
    point.merge(RobotPoint(2));
    EXPECT_NEAR(point.get_x(), 2.5, PRECISION_LENGTH);
    EXPECT_NEAR(point.get_y(), 2, PRECISION_LENGTH);
    EXPECT_TRUE(isnan(point.get_angle()));
    point.set_x(NAN);
    point.set_y(NAN);
    point.set_angle(1);
    point.merge(RobotPoint(NAN, 1, 2));
    EXPECT_TRUE(isnan(point.get_x()));
    EXPECT_NEAR(point.get_y(), 1, PRECISION_LENGTH);
    EXPECT_NEAR(point.get_angle(), 1.5, PRECISION_LENGTH);
}