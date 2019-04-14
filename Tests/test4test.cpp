//
// Created by Danila on 16.03.2019.
//

#include "test.h"

TEST(Test4Test, ReadPointsVectoreFromFile) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MaxCorners_5k.ld", points));
    ASSERT_EQ(points.size(), 5000);
    EXPECT_NEAR(points[0].get_r(), 348.2209, PRECISION_LENGTH * 0.1);
    EXPECT_NEAR(points[0].get_f(), 0, PRECISION_ANGLE * 0.1);
    EXPECT_NEAR(points[4999].get_r(), 347.1262, PRECISION_LENGTH * 0.1);
    EXPECT_NEAR(points[4999].get_f(), 6.281929, PRECISION_ANGLE * 0.1);
}

TEST(Test4Test, Lines2String) {
    std::vector<std::vector<Point>> p = {{Point(0.12345, 0), Point(1)}, {Point(-3, 2)}};
    ASSERT_EQ(lines2string(p),
              "{{(0.123000, 0.000000), (1.000000, nan)}, {(-3.000000, 2.000000)}}");

}
