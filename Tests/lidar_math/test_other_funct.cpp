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

TEST(MathFunc, Sign) {
    EXPECT_EQ(0, sign(0));
    EXPECT_EQ(-1, sign(-0.123));
    EXPECT_EQ(1, sign(10.123));
}

TEST(MathFunc, PositionRelativeLine) {
    EXPECT_EQ(position_relative_line({1, 1}, {-1, -1}, {0.1, 0}, 0.2), 0);
    EXPECT_EQ(position_relative_line({1, 1}, {-1, -1}, {2, 3}, 0), 1);
    EXPECT_EQ(position_relative_line({1, 1}, {-1, -1}, {2, -3}, 0), -1);
    EXPECT_EQ(position_relative_line({-3, 2}, {0, 0}, {0, 2}, 0.1), 1);
    EXPECT_EQ(position_relative_line({-3, 2}, {0, 0}, {0, -1}, 0), -1);
}

TEST(MathFunc, GetLinesCorne) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//6.ld", points));
    auto corners = get_corners(points);
    {
        DebugFieldMat mat;
        mat.add_lines(corners);
        show_debug_img("", mat);
    }
    double ang = get_angle_lines(corners, {Point{-1000},
                                           Point{-1000}}, 200);
    std::cout << ang * 180 / M_PI << std::endl;
    corners_rot(corners, ang);
    {
        DebugFieldMat mat;
        mat.add_lines(corners);
        show_debug_img("", mat);
    }
}

TEST(PositiLeftCorne, 1) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//12.ld", points));
    auto c = position_box_left_corners(points, 1, 1, Point{-50, 115 * 3}, show_debug_img);
    std::cout << "x:" << c.get_x() << " y: " << c.get_y() << std::endl;
}
