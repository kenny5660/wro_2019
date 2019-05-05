//
// Created by Danila on 05.05.2019.
//

#include "../test.h"

TEST(LidaDataFilter, Filter) {
    std::vector<PolarPoint> points = {{1, 0}, {0, 0.416346}, {0, 0.832693}, {3.1622776601684, 1.2490457723983}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {8.0622577482985, 1.4464413322481}};
    data_filter(points, 3);
    ASSERT_EQ(points.size(), 7);
    EXPECT_NEAR(points[0].get_r(), 1, PRECISION_LENGTH);
    EXPECT_NEAR(points[0].get_f(), 0, PRECISION_ANGLE);
    EXPECT_NEAR(points[1].get_r(), 3.1622776601684, PRECISION_LENGTH);
    EXPECT_NEAR(points[2].get_r(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(points[3].get_r(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(points[4].get_r(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(points[5].get_r(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(points[6].get_r(), 8.0622577482985, PRECISION_LENGTH);
}