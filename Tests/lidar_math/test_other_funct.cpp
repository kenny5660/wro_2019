//
// Created by Danila on 04.04.2019.
//

#include "../test.h"

TEST(MathFunc, LineCross) {
    Point p = get_line_cross(Point(0, 0), Point(2, 0), Point(1, 1), Point(1, -1));
    EXPECT_NEAR(p.get_x(), 1, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), 0, PRECISION_LENGTH);
    p = get_line_cross(Point(1, 0), Point(0, 1), Point(2, -1), Point(2, 1));
    EXPECT_TRUE(std::isnan(p.get_x()));
    EXPECT_TRUE(std::isnan(p.get_y()));
    p = get_line_cross(Point(0, 0), Point(0, -5), Point(1, -1), Point(-1, -3));
    EXPECT_NEAR(p.get_x(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), -2, PRECISION_LENGTH);
}

TEST(MathFunc, PoinyInOutline) {
    EXPECT_TRUE(in_outline({{0, 0}, {0, 1}, {1, 1}, {1, 0}}, {0.5, 0.5}));
    EXPECT_TRUE(in_outline({{-1, -1},
                            {0, 0},
                            {-1, 1},
                            {1, 1},
                            {1, -1}}, {-0.5, 0.75}));
    EXPECT_TRUE(in_outline({{-1, -1},
                            {0, 0},
                            {-1, 1},
                            {1, 1},
                            {1, -1}}, {-0.5, -0.75}));
    EXPECT_FALSE(in_outline({{-1, -1},
                            {0, 0},
                            {-1, 1},
                            {1, 1},
                            {1, -1}}, {-0.5, 0.25}));
}
