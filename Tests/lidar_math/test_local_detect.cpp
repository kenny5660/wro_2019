//
// Created by Danila on 18.05.2019.
//

#include "../test.h"

TEST(PositionBoxSide, l) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real/PositionFromBox/Box1.ld", points));
    data_filter(points);
    {
        std::vector<Point> dp;
        for (int i = 0; i < points.size(); i++) {
            dp.push_back(points[i].to_cartesian(-M_PI, true));
        }
        DebugFieldMat mat;
        add_points_img(mat, dp);
        show_debug_img("", mat);
    }
    //Point p = position_box_left_corners(points, 1, 1, show_debug_img);
    //EXPECT_NEAR(p.get_x(), 9, 10);
    //EXPECT_NEAR(p.get_y(), 140, 10);
}

//TEST(PositionBoxSide, 2) {
//    std::vector<PolarPoint> points;
//    ASSERT_FALSE(read("Real/PositionFromBox/Box2.ld", points));
//    data_filter(points);
//    {
//        std::vector<Point> dp;
//        for (int i = 0; i < points.size(); i++) {
//            dp.push_back(points[i].to_cartesian(-M_PI, true));
//        }
//        DebugFieldMat mat;
//        add_points_img(mat, dp);
//        show_debug_img("", mat);
//    }
////    Point p = position_box_side(points, 1, 1, show_debug_img);
////    EXPECT_NEAR(p.get_x(), 9, 10);
////    EXPECT_NEAR(p.get_y(), 140, 10);
//}

