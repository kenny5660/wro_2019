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

TEST(GetCornersFromObj, GetCorners) {
    std::vector<Point> points = {
        {0, -2},
        {0, -1},
        {0, 0},
        {1, 0},
        {2, 0},
        {3, 0}
    };
    std::vector<Point> ans {points[0]};
    get_corners_from_obj(points, 0, points.size() - 1, ans, 1.5);
    ASSERT_EQ(ans.size(), 3);
    EXPECT_NEAR(ans[0].get_x(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0].get_y(), -2, PRECISION_LENGTH);
    EXPECT_NEAR(ans[1].get_x(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[1].get_y(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[2].get_x(), 3, PRECISION_LENGTH);
    EXPECT_NEAR(ans[2].get_y(), 0, PRECISION_LENGTH);
}

TEST(GetCornersFromObj, CrossWithMainLineInCenter) {
    std::vector<Point> points = {
        {0, 0},
        {0, 1},
        {0, 2},
        {0, 3},
        {0, 4},
        {1, 4},
        {2, 4},
        {3, 4},
        {4, 4},
        {4, 5},
        {4, 6},
        {4, 7},
        {4, 8},
        {5, 8},
        {6, 8},
        {7, 8},
        {8, 8}
    };
    std::vector<Point> ans {points[0]};
    get_corners_from_obj(points, 0, points.size() - 1, ans, 1.1);
    std::cout << ans.size();
    ASSERT_EQ(ans.size(), 5);
    EXPECT_NEAR(ans[0].get_x(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0].get_y(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[1].get_x(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[1].get_y(), 4, PRECISION_LENGTH);
    EXPECT_NEAR(ans[2].get_x(), 4, PRECISION_LENGTH);
    EXPECT_NEAR(ans[2].get_y(), 4, PRECISION_LENGTH);
    EXPECT_NEAR(ans[3].get_x(), 4, PRECISION_LENGTH);
    EXPECT_NEAR(ans[3].get_y(), 8, PRECISION_LENGTH);
    EXPECT_NEAR(ans[4].get_x(), 8, PRECISION_LENGTH);
    EXPECT_NEAR(ans[4].get_y(), 8, PRECISION_LENGTH);
}

TEST(GetCornersFromObj, TwoPartDivorsedBy1Line) {
    std::vector<Point> points = {
        {0, 1},
        {0, 2},
        {0, 3},
        {0, 4},
        {0, 5},
        {0, 6},
        {0, 7},
        {0, 8},
        {0, 9},
        {0, 10},
        {1, 10},
        {2, 10},
        {3, 10},
        {4, 10},
        {5, 10},
        {6, 10},
        {7, 10},
        {8, 10},
        {9, 10},
        {10, 10},
        {10, 9},
        {10, 8},
        {10, 7},
        {10, 6},
        {11, 6},
        {12, 6},
        {13, 6},
        {14, 6},
        {14, 7},
        {14, 8},
    };
    std::vector<Point> ans {points[0]};
    get_corners_from_obj(points, 0, points.size() - 1, ans, 1.01);
    ASSERT_EQ(ans.size(), 6);
    EXPECT_NEAR(ans[0].get_x(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0].get_y(), 1, PRECISION_LENGTH);
    EXPECT_NEAR(ans[1].get_x(), 0, PRECISION_LENGTH);
    EXPECT_NEAR(ans[1].get_y(), 10, PRECISION_LENGTH);
    EXPECT_NEAR(ans[2].get_x(), 10, PRECISION_LENGTH);
    EXPECT_NEAR(ans[2].get_y(), 10, PRECISION_LENGTH);
    EXPECT_NEAR(ans[3].get_x(), 10, PRECISION_LENGTH);
    EXPECT_NEAR(ans[3].get_y(), 6, PRECISION_LENGTH);
    EXPECT_NEAR(ans[4].get_x(), 14, PRECISION_LENGTH);
    EXPECT_NEAR(ans[4].get_y(), 6, PRECISION_LENGTH);
    EXPECT_NEAR(ans[5].get_x(), 14, PRECISION_LENGTH);
    EXPECT_NEAR(ans[5].get_y(), 8, PRECISION_LENGTH);
}

TEST(GetCornerSimulator, Line) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Line_881.ld", points));
    std::vector<std::vector<Point>> ans = get_corners(points);
    ASSERT_EQ(ans.size(), 1);
    ASSERT_EQ(ans[0].size(), 2);
    EXPECT_NEAR(ans[0][0].get_x(), -936.862, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][0].get_y(), -333.970, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][1].get_x(), -935.259, PRECISION_LENGTH);
    EXPECT_NEAR(ans[0][1].get_y(), 340.037, PRECISION_LENGTH);
}

TEST(GetCornerSimulator, Corner) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Corner_2001.ld", points));
    std::vector<std::vector<Point>> ans = get_corners(points);
    ASSERT_EQ(ans.size(), 1);
    ASSERT_EQ(ans[0].size(), 3);
    EXPECT_NEAR(ans[0][0].get_x(), -480, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][0].get_y(), 0, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][1].get_x(), -480, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][1].get_y(), 480, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][2].get_x(), 0, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][2].get_y(), 480, PRECISION_REAL_LENGTH);
}

TEST(GetCornerSimulator, FreeField) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("FreeField_5k.ld", points));
    std::vector<std::vector<Point>> ans = get_corners(points);
    ASSERT_EQ(ans.size(), 2);
    ASSERT_EQ(ans[0].size(), 6);
    ASSERT_EQ(ans[1].size(), 2);
    EXPECT_NEAR(ans[0][0].get_x(), -1200.3682000951972, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][0].get_y(), 644.27817080871773, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][1].get_x(), -1200, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][1].get_y(), 1200, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][2].get_x(), 1200, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][2].get_y(), 1200, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][3].get_x(), 1200, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][3].get_y(), -1200, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][4].get_x(), -1200, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][4].get_y(), -1200, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][5].get_x(), -1200, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][5].get_y(), -308.33047736102998, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[1][0].get_x(), -480, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[1][0].get_y(), -122.5, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[1][1].get_x(), -480, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[1][1].get_y(), 256.5, PRECISION_REAL_LENGTH);
};

TEST(GetCornerSimulator, MaxCornerAndRoot) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MaxCorners_5k.ld", points));
    std::vector<std::vector<Point>> ans = get_corners(points);
    ASSERT_EQ(ans.size(), 13);
    ASSERT_EQ(ans[0].size(), 3);
    ASSERT_EQ(ans[1].size(), 2);
    ASSERT_EQ(ans[2].size(), 3);
    ASSERT_EQ(ans[4].size(), 3);
    ASSERT_EQ(ans[5].size(), 2);
    ASSERT_EQ(ans[6].size(), 4);
    ASSERT_EQ(ans[7].size(), 3);
    ASSERT_EQ(ans[8].size(), 3);
    ASSERT_EQ(ans[9].size(), 3);
    ASSERT_EQ(ans[10].size(), 2);
    ASSERT_EQ(ans[11].size(), 3);
    ASSERT_EQ(ans[12].size(), 4);

    EXPECT_NEAR(ans[0][0].get_x(), -689.52043611642318, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][0].get_y(), 850.75785786296296, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][1].get_x(), -527.02212758340374, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][1].get_y(), 688.31837515518373, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][2].get_x(), -358.60018904916109, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[0][2].get_y(), 858.10100728266013, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[1][0].get_x(), -507.08638800927105, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[1][0].get_y(), 1217.7165494417004, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[1][1].get_x(), -302.82369180569265, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[1][1].get_y(), 1421.7382786355338, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[2][0].get_x(), -271.20062630592184, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[2][0].get_y(), 1281.1780740191919, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[2][1].get_x(), -103.76569784926748, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[2][1].get_y(), 1112.6459653100192, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[2][2].get_x(), 66.132083604965032, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[2][2].get_y(), 1282.4669383359853, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[3][0].get_x(), 96.183507735845097, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[3][0].get_y(), 1820.7355397970675, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[3][1].get_x(), 660.58563356138461, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[3][1].get_y(), 2385.2397031093678, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[3][2].get_x(), 992.13169748063524, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[3][2].get_y(), 2054.5261165760908, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[4][0].get_x(), 662.02054354215034, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[4][0].get_y(), 1366.5325852298276, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[4][1].get_x(), 829.56543970030668, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[4][1].get_y(), 1197.8669541736581, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[4][2].get_x(), 995.92797761540146, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[4][2].get_y(), 1363.5605951579623, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[5][0].get_x(), 1287.3283742596539, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[5][0].get_y(), 1757.8801083905, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[5][1].get_x(), 1491.1195720006272, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[5][1].get_y(), 1556.2284181242503, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[6][0].get_x(), 1165.1381866631473, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[6][0].get_y(), 1212.9598357010645, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[6][1].get_x(), 1171.3957570882399, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[6][1].get_y(), 1198.1957718693675, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[6][2].get_x(), 1495.0845059229771, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[6][2].get_y(), 1304.7992140458769, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[6][3].get_x(), 1573.3019037501072, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[6][3].get_y(), 1072.1107847556841, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[7][0].get_x(), 1279.0485533985252, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[7][0].get_y(), 869.24059166651307, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[7][1].get_x(), 1288.8879892183384, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[7][1].get_y(), 854.77917855478836, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[7][2].get_x(), 1616.075225958338, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[7][2].get_y(), 963.98592964820136, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[8][0].get_x(), 1909.5535897282944, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[8][0].get_y(), 1135.795898006163, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[8][1].get_x(), 2355.7266509048918, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[8][1].get_y(), 687.61236889924544, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[8][2].get_x(), 1804.5138933106005, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[8][2].get_y(), 136.31814820217349, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[9][0].get_x(), 1252.3671768000977, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[9][0].get_y(), 93.023084713380257, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[9][1].get_x(), 1084.3550654928233, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[9][1].get_y(), -75.064051516455052, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[9][2].get_x(), 1254.3974383929753, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[9][2].get_y(), -244.19354941959702, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[10][0].get_x(), 1394.5131338440474, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[10][0].get_y(), -273.28964985980389, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[10][1].get_x(), 1256.471201700499, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[10][1].get_y(), -411.74468216901641, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[11][0].get_x(), 744.01356142287136, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[11][0].get_y(), -244.8496008056685, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[11][1].get_x(), 575.60716687413878, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[11][1].get_y(), -413.79885690896657, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[11][2].get_x(), 738.38777055918024, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[11][2].get_y(), -577.22266932989851, PRECISION_REAL_LENGTH);

    EXPECT_NEAR(ans[12][0].get_x(), 935.82497640610961, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[12][0].get_y(), -733.46306694799193, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[12][1].get_x(), 658.36895318531583, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[12][1].get_y(), -1006.3992315733511, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[12][2].get_x(), -1033.423763163338, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[12][2].get_y(), 685.35806610784482, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[12][3].get_x(), -773.48064384421389, PRECISION_REAL_LENGTH);
    EXPECT_NEAR(ans[12][3].get_y(), 951.90076368413008, PRECISION_REAL_LENGTH);
};
