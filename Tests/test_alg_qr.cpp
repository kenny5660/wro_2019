//
// Created by Danila on 19.05.2019.
//

#include "test.h"
#include "../highLogic/alg.h"

extern PolarPoint get_box_color_point(const std::vector<PolarPoint> &points, const RobotPoint &position, BoxMap &box);

TEST(Alg, 1) {
    Robot robot;

    do_alg_code(robot, false, "(K,G,J,D)(F,H,D,J)(S,F,U,H)(I,Q,G,S)");
    //do_alg_code(robot, false, "(F,J,H,H)(I,R,K,T)(O,A,M,C)(Q,Q,O,S)");
}

TEST(Alg, 2) {
    RobotPoint robot(850, 1552.5, 0);
    std::vector<PolarPoint> points;
    read("14_08_19.ld", points);
    BoxMap box(Point{115, 690});
    get_box_color_point(points, robot, box);
}
