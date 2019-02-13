//
// Created by Danila on 13.02.2019.
//

#include "test.h"

TEST(PositionFromDownLine, PerpendicularLeftTriangle) {
    RobotPoint p = get_coordinates_from_line(13.217, 9.378, 0.68783426, 3.24735961, bottom_field_margin);
    EXPECT_TRUE(isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), (MAX_FIELD_HEIGHT - 9.36), PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), 0.67806041, PRECISION_ANGLE);
}

TEST(PositionFromDownLine, PerpendicularInTriangle) {
    RobotPoint p = get_coordinates_from_line(13.145, 12.526, 0.8841838, 2.41501209, bottom_field_margin);
    EXPECT_TRUE(isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), (MAX_FIELD_HEIGHT - 11.58), PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), 1.804386, PRECISION_ANGLE);
}

TEST(PositionFromDownLine, PerpendicularRightTriangle) {
    RobotPoint p = get_coordinates_from_line(21.933, 30.99, 0.38117991, 3.29378536, bottom_field_margin);
    EXPECT_TRUE(isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), (MAX_FIELD_HEIGHT - 18.867), PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), 1.95372156, PRECISION_ANGLE);
}

TEST(PositionFromDownLine, PerpendicularIsCParametr) {
    RobotPoint p = get_coordinates_from_line(28.765, 20.270, 0.7888888, -1.1718141, bottom_field_margin);
    EXPECT_TRUE(isnan(p.get_x()));
    EXPECT_NEAR(p.get_y(), (MAX_FIELD_HEIGHT - 20.27), PRECISION_LENGTH);
    EXPECT_NEAR(p.get_angle(), 5.09531422, PRECISION_ANGLE);
}
