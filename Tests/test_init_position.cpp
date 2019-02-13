//
// Created by Danila on 13.02.2019.
//

#include "test.h"

TEST(InitPositionFromLine, LookUp) {
    RobotPoint p = init_position_from_line(11.181, 8.151, 0.523599);
    EXPECT_NEAR(p.get_x(), 7.861, PRECISION_LENGTH);
    EXPECT_TRUE(isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), 5.75382695, PRECISION_ANGLE);
}

TEST(InitPositionFromLine, LookDown) {
    RobotPoint p = init_position_from_line(17.033, 38.489, 0.349066);
    EXPECT_NEAR(p.get_x(), 9.654, PRECISION_LENGTH);
    EXPECT_TRUE(isnan(p.get_y()));
    EXPECT_NEAR(p.get_angle(), 1.1426671, PRECISION_ANGLE);
}

TEST(InitPositionFromLine, LookPerpendicular) {
    RobotPoint p = init_position_from_line(44.487, 44.487, 0.244346);
    EXPECT_NEAR(p.get_x(), 44.155, PRECISION_LENGTH);
    EXPECT_TRUE(isnan(p.get_y()));
    EXPECT_DOUBLE_EQ(p.get_angle(), 0);
}
