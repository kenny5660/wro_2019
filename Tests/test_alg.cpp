//
// Created by Danila on 18.06.2019.
//

#include "test.h"
#include "../highLogic/alg.h"

TEST(TestDetectPosition, 1) {
    Robot robot;
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//8.ld", points));
    detect_position(robot, points, 300);
}
