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
    RobotPoint robot(9 * 115, 5 * 115, 0);
    std::vector<PolarPoint> points;
    read("15_08_19_1.ld", points);
    BoxMap box(Point{8 * 115, 115});
    auto p = get_box_color_point(points, robot, box);
    std::cout << p.get_r() << " " << p.get_f() << std::endl;
  DebugFieldMat mat;
  std::vector<Point> buff;
    for (auto i : points) {
          buff.push_back(i.to_cartesian(-M_PI, true));
    }
    add_points_img(mat, buff);
  add_points_img(mat, {p.to_cartesian(-M_PI, true)}, {255, 255, 255});
  add_points_img(mat, {{0, 3 * 115}}, {255, 0, 0});

  imshow("", mat);
    cv::waitKey();
}
