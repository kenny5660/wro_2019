//
// Created by Danila on 15.02.2019.
//

#include "..\test.h"

TEST(GetGroupsOfObj, Line) {
    std::vector<Point> points = {
        {0, 0},
        {0, 1},
        {0, 2},
        {0, 3},
        {0, 4},
        {0, 5}
    };
    auto ans = get_groups_obj(points, 1.5);
    ASSERT_EQ(ans.size(), 1);
    ASSERT_EQ(ans[0].size(), 6);
    for (int i = 0; i < points.size(); i++)
        EXPECT_NEAR(ans[0][i].get_x(), 0, PRECISION_LENGTH);
    for (int i = 0; i < points.size(); i++)
        EXPECT_NEAR(ans[0][i].get_y(), i, PRECISION_LENGTH);
}

TEST(GetGroupsOfObj, Merger) {
    std::vector<Point> points = {
        {0, 0},
        {0, 1},
        {0, 2},
        {0, -3},
        {0, -2},
        {0, -1}
    };
    auto ans = get_groups_obj(points, 1.5);
    ASSERT_EQ(ans.size(), 1);
    ASSERT_EQ(ans[0].size(), 6);
    for (int i = 0; i < points.size(); i++)
        EXPECT_NEAR(ans[0][i].get_x(), 0, PRECISION_LENGTH);
    for (int i = 0; i < points.size(); i++)
        EXPECT_NEAR(ans[0][i].get_y(), i - 3, PRECISION_LENGTH);
}

TEST(GetGroupsOfObj, TwoPointStart) {
    std::vector<Point> points = {
        {0, 0},
        {0, 1},
        {0, -4},
        {0, -3},
        {0, -2},
        {0, -1}
    };
    auto ans = get_groups_obj(points, 1.5);
    ASSERT_EQ(ans.size(), 1);
    ASSERT_EQ(ans[0].size(), 6);
    for (int i = 0; i < points.size(); i++)
        EXPECT_NEAR(ans[0][i].get_x(), 0, PRECISION_LENGTH);
    for (int i = 0; i < points.size(); i++)
        EXPECT_NEAR(ans[0][i].get_y(), i - 4, PRECISION_LENGTH);
}

TEST(GetGroupsOfObj, OnePointDown) {
    std::vector<Point> points = {
        {0, 0},
        {0, 1},
        {0, 2},
        {0, 3},
        {0, 4},
        {0, -1}
    };
    auto ans = get_groups_obj(points, 1.5);
    ASSERT_EQ(ans.size(), 1);
    ASSERT_EQ(ans[0].size(), 6);
    for (int i = 0; i < points.size(); i++)
        EXPECT_NEAR(ans[0][i].get_x(), 0, PRECISION_LENGTH);
    for (int i = 0; i < points.size(); i++)
        EXPECT_NEAR(ans[0][i].get_y(), i - 1, PRECISION_LENGTH);
}

TEST(GetGroupsOfObj, Square) {
    std::vector<Point> points = {
        {0, 0},
        {0, 1},
        {1, 1},
        {1, 0}
    };
    auto ans = get_groups_obj(points, 1.5);
    ASSERT_EQ(ans.size(), 1);
    ASSERT_EQ(ans[0].size(), 4);
    EXPECT_NEAR(ans[0][0].get_x(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][0].get_y(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][1].get_x(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][1].get_y(), 1, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][2].get_x(), 1, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][2].get_y(), 1, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][3].get_x(), 1, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][3].get_y(), 0, PRECISION_LENGTH);
}

TEST(GetGroupsOfObj, Split) {
    std::vector<Point> points = {
        {0, -1},
        {0, 0},
        {1, 1},
        {1, 2},
        {1, 3},
        {1, 4}
    };
    auto ans = get_groups_obj(points, 1.1);
    ASSERT_EQ(ans.size(), 2);
    ASSERT_EQ(ans[0].size(), 2);
    ASSERT_EQ(ans[1].size(), 4);
    for (int i = 0; i < 2; i++)
        EXPECT_NEAR(ans[0][i].get_x(), 0, PRECISION_LENGTH);
    for (int i = 2; i < points.size(); i++)
        EXPECT_NEAR(ans[1][i - 2].get_x(), 1, PRECISION_LENGTH);
    for (int i = 0; i < 2; i++)
        EXPECT_NEAR(ans[0][i].get_y(), i - 1, PRECISION_LENGTH);
    for (int i = 2; i < points.size(); i++)
        EXPECT_NEAR(ans[1][i - 2].get_y(), i - 1, PRECISION_LENGTH);
}

TEST(GetCornersFromObj, GetLine) {
    std::vector<Point> points = {
        {-3, -2},
        {-2, -1},
        {-1, 0},
        {0, 1},
        {1, 2},
        {2, 3}
    };
    std::vector<Point> ans {points[0]};
    get_corners_from_obj(points, 0, points.size() - 1, ans, 1.5);
    ASSERT_EQ(ans.size(), 2);
    EXPECT_NEAR(ans[0].get_x(), -3, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0].get_y(), -2, PRECISION_LENGTH);
    EXPECT_NEAR(ans[1].get_x(), 2, PRECISION_LENGTH);
    EXPECT_NEAR(ans[1].get_y(), 3, PRECISION_LENGTH);
}

//TEST(GetCorner, OnlyCorners) {
//    std::vector<PolarPoint> points(4);
//    double robot_rotation = 89;
//    points[0] = PolarPoint(1390.16359102, degree2radian(robot_rotation + 0));
//    points[1] = PolarPoint(1414.21356237, degree2radian(robot_rotation + 1));
//    points[2] = PolarPoint(1390.16359102, degree2radian(robot_rotation + 2));
//    points[3] = PolarPoint(1367.3274611, degree2radian(robot_rotation + 3));
//    auto corner_points = get_corners(points);
//    ASSERT_EQ(corner_points.size(), 1);
//    ASSERT_EQ(corner_points[0].size(), 3);
//    EXPECT_NEAR(corner_points[0][0].get_x(), -24.2617000, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][0].get_y(), 1389.9518624, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_x(), 0, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_y(), 1414.21356237, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_x(), 47.7190402, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_y(), 1366.4945222, PRECISION_LENGTH);
//}
//
//TEST(GetCorner, DivideOnLine) {
//    std::vector<PolarPoint> points(5);
//    double robot_rotation = 0;
//    points[0] = PolarPoint(10, degree2radian(robot_rotation + 0));
//    points[1] = PolarPoint(11.5470053838, degree2radian(robot_rotation + 30));
//    points[2] = PolarPoint(500, degree2radian(robot_rotation + 90));
//    points[3] = PolarPoint(11.5470053838, degree2radian(robot_rotation + 150));
//    points[4] = PolarPoint(10, degree2radian(robot_rotation + 180));
//    auto corner_points = get_corners(points, degree2radian(35));
//    ASSERT_EQ(corner_points.size(), 2);
//    ASSERT_EQ(corner_points[0].size(), 2);
//    ASSERT_EQ(corner_points[1].size(), 2);
//    EXPECT_NEAR(corner_points[0][0].get_x(), -10, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][0].get_y(), 0, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_x(), -10, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_y(), 5.7735026919, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[1][0].get_x(), 10, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[1][0].get_y(), 5.7735026919, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[1][1].get_x(), 10, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[1][1].get_y(), 0, PRECISION_LENGTH);
//}
//
//TEST(GetCorner, combineEndsWithOnePoints) {
//    std::vector<PolarPoint> points(2);
//    double robot_rotation = 0;
//    points[0] = PolarPoint(15, degree2radian(robot_rotation + 0));
//    points[1] = PolarPoint(15.0022849207, degree2radian(robot_rotation + 359));
//    auto corner_points = get_corners(points);
//    ASSERT_EQ(corner_points.size(), 1);
//    ASSERT_EQ(corner_points[0].size(), 2);
//    EXPECT_NEAR(corner_points[0][0].get_x(), -15, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][0].get_y(), -0.26182597392, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_x(), -15, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_y(), 0, PRECISION_LENGTH);
//}
//
//TEST(GetCorner, combineEndsWithEndOnePoint) {
//    std::vector<PolarPoint> points(4);
//    double robot_rotation = 0;
//    points[0] = PolarPoint(1500, degree2radian(robot_rotation + 0));
//    points[1] = PolarPoint(1500.22849207, degree2radian(robot_rotation + 1));
//    points[2] = PolarPoint(750.228509459, degree2radian(robot_rotation + 2));
//    points[3] = PolarPoint(1500.22849207, degree2radian(robot_rotation + 359));
//    auto corner_points = get_corners(points);
//    ASSERT_EQ(corner_points.size(), 1);
//    ASSERT_EQ(corner_points[0].size(), 3);
//    EXPECT_NEAR(corner_points[0][0].get_x(), -1500, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][0].get_y(), -26.182597392, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_x(), -1500, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_y(), 26.182597392, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_x(), -749.771490522, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_y(), 26.182597392, PRECISION_LENGTH);
//}
//
//TEST(GetCorner, combineEndsWithFierstOnePoint) {
//    std::vector<PolarPoint> points(4);
//    double robot_rotation = 0;
//    points[0] = PolarPoint(1500, degree2radian(robot_rotation + 0));
//    points[1] = PolarPoint(1000.55867024, degree2radian(robot_rotation + 357));
//    points[2] = PolarPoint(1500.91431645, degree2radian(robot_rotation + 358));
//    points[3] = PolarPoint(1500.22849207, degree2radian(robot_rotation + 359));
//    auto corner_points = get_corners(points);
//    ASSERT_EQ(corner_points.size(), 1);
//    ASSERT_EQ(corner_points[0].size(), 3);
//    EXPECT_NEAR(corner_points[0][0].get_x(), -999.187439353, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][0].get_y(), -2 * 26.182597392, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_x(), -1500, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_y(), -2 * 26.182597392, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_x(), -1500, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_y(), 0, PRECISION_LENGTH);
//}
//
//TEST(GetCorner, combineEnds) {
//    std::vector<PolarPoint> points(6);
//    double robot_rotation = 0;
//    points[0] = PolarPoint(1500, degree2radian(robot_rotation + 0));
//    points[1] = PolarPoint(1500.22849207, degree2radian(robot_rotation + 1));
//    points[2] = PolarPoint(750.228509459, degree2radian(robot_rotation + 2));
//    points[3] = PolarPoint(1000.55867024, degree2radian(robot_rotation + 357));
//    points[4] = PolarPoint(1500.91431645, degree2radian(robot_rotation + 358));
//    points[5] = PolarPoint(1500.22849207, degree2radian(robot_rotation + 359));
//    auto corner_points = get_corners(points);
//    ASSERT_EQ(corner_points.size(), 1);
//    ASSERT_EQ(corner_points[0].size(), 4);
//    EXPECT_NEAR(corner_points[0][0].get_x(), -999.187439353, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][0].get_y(), -2 * 26.182597392, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_x(), -1500, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_y(), -2 * 26.182597392, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_x(), -1500, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_y(), 26.182597392, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][3].get_x(), -749.771490522, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][3].get_y(), 26.182597392, PRECISION_LENGTH);
//}
//
//
//TEST(GetCorner, detectedLineFromManyPoints) {
//    std::vector<PolarPoint> points{
//        { 599.9465, 0.000000 },
//        { 599.3135, 0.017453 },
//        { 600.6564, 0.034907 },
//        { 600.0567, 0.052360 },
//        { 601.9774, 0.069813 },
//        { 602.6678, 0.087266 },
//        { 603.8598, 0.104720 },
//        { 603.7405, 0.122173 },
//        { 605.2700, 0.139626 },
//        { 607.3963, 0.157080 },
//        { 608.3443, 0.174533 },
//        { 610.8113, 0.191986 },
//        { 612.9887, 0.209440 },
//        { 615.3235, 0.226893 },
//        { 619.1719, 0.244346 },
//        { 621.4420, 0.261799 },
//        { 624.1670, 0.279253 },
//        { 628.2854, 0.296706 },
//        { 629.9797, 0.314159 },
//        { 633.9494, 0.331613 },
//        { 638.4098, 0.349066 },
//        { 643.6791, 0.366519 },
//        { 647.2693, 0.383972 },
//        { 651.7929, 0.401426 },
//        { 656.7908, 0.418879 },
//        { 661.5139, 0.436332 },
//        { 668.1600, 0.453786 },
//        { 673.3242, 0.471239 },
//        { 679.2686, 0.488692 },
//        { 685.4131, 0.506145 },
//        { 692.4810, 0.523599 },
//        { 699.1756, 0.541052 },
//        { 707.6289, 0.558505 },
//        { 714.5903, 0.575959 },
//        { 723.1677, 0.593412 },
//        { 733.3983, 0.610865 },
//        { 740.9544, 0.628319 },
//        { 752.2103, 0.645772 },
//        { 762.3321, 0.663225 },
//        { 772.5138, 0.680678 },
//        { 783.9364, 0.698132 },
//        { 795.7469, 0.715585 },
//        { 808.0933, 0.733038 },
//        { 819.4958, 0.750492 },
//        { 834.9698, 0.767945 },
//        { 848.9698, 0.785398 },
//        { 863.2939, 0.802851 },
//        { 879.3820, 0.820305 },
//        { 896.9176, 0.837758 },
//        { 915.0303, 0.855211 },
//        { 933.5805, 0.872665 },
//        { 953.7152, 0.890118 },
//        { 974.7020, 0.907571 },
//        { 997.7851, 0.925025 },
//        { 1021.0220, 0.942478 },
//        { 1046.3509, 0.959931 },
//        { 1072.4962, 0.977384 },
//        { 1102.0100, 0.994838 },
//        { 1132.4793, 1.012291 },
//        { 1165.3765, 1.029744 },
//        { 1199.2797, 1.047198 },
//        { 1238.5733, 1.064651 },
//        { 1277.4135, 1.082104 }
//    };
//    auto corner_points = get_corners(points);
//    ASSERT_EQ(corner_points.size(), 1);
//    ASSERT_EQ(corner_points[0].size(), 2);
//    EXPECT_NEAR(corner_points[0][0].get_x(), -599.9465, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][0].get_y(), 0, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_x(), -599.71, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_y(), 1127.89, PRECISION_LENGTH);
//}
//
//TEST(GetCorner, detectedAndCombineLineFromManyPoints) {
//    std::vector<PolarPoint> points{
//        { 599.9465, 0.000000 },
//        { 599.3135, 0.017453 },
//        { 600.6564, 0.034907 },
//        { 600.0567, 0.052360 },
//        { 601.9774, 0.069813 },
//        { 602.6678, 0.087266 },
//        { 603.8598, 0.104720 },
//        { 603.7405, 0.122173 },
//        { 605.2700, 0.139626 },
//        { 607.3963, 0.157080 },
//        { 608.3443, 0.174533 },
//        { 610.8113, 0.191986 },
//        { 612.9887, 0.209440 },
//        { 615.3235, 0.226893 },
//        { 619.1719, 0.244346 },
//        { 621.4420, 0.261799 },
//        { 624.1670, 0.279253 },
//        { 628.2854, 0.296706 },
//        { 629.9797, 0.314159 },
//        { 633.9494, 0.331613 },
//        { 638.4098, 0.349066 },
//        { 643.6791, 0.366519 },
//        { 647.2693, 0.383972 },
//        { 651.7929, 0.401426 },
//        { 656.7908, 0.418879 },
//        { 661.5139, 0.436332 },
//        { 668.1600, 0.453786 },
//        { 673.3242, 0.471239 },
//        { 679.2686, 0.488692 },
//        { 685.4131, 0.506145 },
//        { 692.4810, 0.523599 },
//        { 699.1756, 0.541052 },
//        { 707.6289, 0.558505 },
//        { 714.5903, 0.575959 },
//        { 723.1677, 0.593412 },
//        { 733.3983, 0.610865 },
//        { 740.9544, 0.628319 },
//        { 752.2103, 0.645772 },
//        { 762.3321, 0.663225 },
//        { 772.5138, 0.680678 },
//        { 783.9364, 0.698132 },
//        { 795.7469, 0.715585 },
//        { 808.0933, 0.733038 },
//        { 819.4958, 0.750492 },
//        { 834.9698, 0.767945 },
//        { 848.9698, 0.785398 },
//        { 863.2939, 0.802851 },
//        { 879.3820, 0.820305 },
//        { 896.9176, 0.837758 },
//        { 915.0303, 0.855211 },
//        { 933.5805, 0.872665 },
//        { 953.7152, 0.890118 },
//        { 974.7020, 0.907571 },
//        { 997.7851, 0.925025 },
//        { 1021.0220, 0.942478 },
//        { 1046.3509, 0.959931 },
//        { 1072.4962, 0.977384 },
//        { 1102.0100, 0.994838 },
//        { 1132.4793, 1.012291 },
//        { 1165.3765, 1.029744 },
//        { 1199.2797, 1.047198 },
//        { 1238.5733, 1.064651 },
//        { 1277.4135, 1.082104 },
//        {1164.7855, 5.253441},
//        {1131.8245, 5.270894},
//        {1102.1576, 5.288348},
//        {1073.8786, 5.305801},
//        {1045.7336, 5.323254},
//        {1021.4057, 5.340708},
//        {997.1816, 5.358161},
//        {975.1495, 5.375614},
//        {952.9151, 5.393067},
//        {932.6156, 5.410521},
//        {913.7998, 5.427974},
//        {897.5258, 5.445427},
//        {878.8300, 5.462881},
//        {863.0470, 5.480334},
//        {849.0830, 5.497787},
//        {834.3428, 5.515240},
//        {821.3320, 5.532694},
//        {807.5624, 5.550147},
//        {795.3343, 5.567600},
//        {784.0763, 5.585054},
//        {771.6060, 5.602507},
//        {761.6737, 5.619960},
//        {750.6608, 5.637413},
//        {742.5488, 5.654867},
//        {732.6203, 5.672320},
//        {724.6221, 5.689773},
//        {715.8373, 5.707227},
//        {707.3762, 5.724680},
//        {700.9072, 5.742133},
//        {693.6434, 5.759587},
//        {685.0296, 5.777040},
//        {679.2893, 5.794493},
//        {673.1360, 5.811946},
//        {667.5117, 5.829400},
//        {662.1790, 5.846853},
//        {657.4513, 5.864306},
//        {651.3131, 5.881760},
//        {646.3339, 5.899213},
//        {641.8626, 5.916666},
//        {637.6641, 5.934119},
//        {634.6002, 5.951573},
//        {631.7328, 5.969026},
//        {627.2484, 5.986479},
//        {623.4290, 6.003933},
//        {621.7033, 6.021386},
//        {619.0366, 6.038839},
//        {616.3812, 6.056293},
//        {614.2013, 6.073746},
//        {611.5492, 6.091199},
//        {608.8672, 6.108652},
//        {607.4515, 6.126106},
//        {606.8670, 6.143559},
//        {603.5439, 6.161012},
//        {603.6146, 6.178466},
//        {601.9356, 6.195919},
//        {601.4638, 6.213372},
//        {600.0942, 6.230825},
//        {600.6871, 6.248279},
//        {599.5329, 6.265732}
//    };
//    auto corner_points = get_corners(points);
//    ASSERT_EQ(corner_points.size(), 1);
//    ASSERT_EQ(corner_points[0].size(), 2);
//    EXPECT_NEAR(corner_points[0][0].get_x(), -599.9465, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][0].get_y(), -998.41607204, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_x(), -599.71, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_y(), 1127.89, PRECISION_LENGTH);
//}
//
//TEST(GetCorner, getLeftTopCornerFromManyPoints) {
//    std::vector<PolarPoint> points{
//        {599.9465, 0.000000},
//        {599.3135, 0.017453},
//        {600.6564, 0.034907},
//        {600.0567, 0.052360},
//        {601.9774, 0.069813},
//        {602.6678, 0.087266},
//        {603.8598, 0.104720},
//        {603.7405, 0.122173},
//        {605.2700, 0.139626},
//        {607.3963, 0.157080},
//        {608.3443, 0.174533},
//        {610.8113, 0.191986},
//        {612.9887, 0.209440},
//        {615.3235, 0.226893},
//        {619.1719, 0.244346},
//        {621.4420, 0.261799},
//        {624.1670, 0.279253},
//        {628.2854, 0.296706},
//        {629.9797, 0.314159},
//        {633.9494, 0.331613},
//        {638.4098, 0.349066},
//        {643.6791, 0.366519},
//        {647.2693, 0.383972},
//        {651.7929, 0.401426},
//        {656.7908, 0.418879},
//        {661.5139, 0.436332},
//        {668.1600, 0.453786},
//        {673.3242, 0.471239},
//        {679.2686, 0.488692},
//        {685.4131, 0.506145},
//        {692.4810, 0.523599},
//        {699.1756, 0.541052},
//        {707.6289, 0.558505},
//        {714.5903, 0.575959},
//        {723.1677, 0.593412},
//        {733.3983, 0.610865},
//        {740.9544, 0.628319},
//        {752.2103, 0.645772},
//        {762.3321, 0.663225},
//        {772.5138, 0.680678},
//        {783.9364, 0.698132},
//        {795.7469, 0.715585},
//        {808.0933, 0.733038},
//        {819.4958, 0.750492},
//        {834.9698, 0.767945},
//        {848.9698, 0.785398},
//        {863.2939, 0.802851},
//        {879.3820, 0.820305},
//        {896.9176, 0.837758},
//        {915.0303, 0.855211},
//        {933.5805, 0.872665},
//        {953.7152, 0.890118},
//        {974.7020, 0.907571},
//        {997.7851, 0.925025},
//        {1021.0220, 0.942478},
//        {1046.3509, 0.959931},
//        {1072.4962, 0.977384},
//        {1102.0100, 0.994838},
//        {1132.4793, 1.012291},
//        {1165.3765, 1.029744},
//        {1199.2797, 1.047198},
//        {1238.5733, 1.064651},
//        {1277.4135, 1.082104},
//        {1321.0373, 1.099557},
//        {1335.4249, 1.117011},
//        {1324.4165, 1.134464},
//        {1314.2843, 1.151917},
//        {1303.9551, 1.169371},
//        {1293.4640, 1.186824},
//        {1285.5511, 1.204277},
//        {1277.4038, 1.221730},
//        {1270.0466, 1.239184},
//        {1261.7134, 1.256637},
//        {1254.1817, 1.274090},
//        {1247.8110, 1.291544},
//        {1241.7010, 1.308997},
//        {1236.8949, 1.326450},
//        {1231.3489, 1.343904},
//        {1227.5385, 1.361357},
//        {1222.8415, 1.378810},
//        {1218.6052, 1.396263},
//        {1214.5321, 1.413717},
//        {1211.8649, 1.431170},
//        {1208.6988, 1.448623},
//        {1205.9243, 1.466077},
//        {1203.8105, 1.483530},
//        {1203.4923, 1.500983},
//        {1201.0602, 1.518436},
//        {1200.1555, 1.535890},
//        {1200.3416, 1.553343},
//        {1199.6968, 1.570796}
//    };
//    auto corner_points = get_corners(points);
//    ASSERT_EQ(corner_points.size(), 1);
//    ASSERT_EQ(corner_points[0].size(), 3);
//    EXPECT_NEAR(corner_points[0][0].get_x(), -599.9465, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][0].get_y(), 0, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_x(), -600, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][1].get_y(), 1200, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_x(), 0, PRECISION_LENGTH);
//    EXPECT_NEAR(corner_points[0][2].get_y(), 1200, PRECISION_LENGTH);
//}
