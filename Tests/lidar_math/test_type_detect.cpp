//
// Created by Danila on 18.03.2019.
//

#include "../test.h"

TEST(CoverLines, TwoCover) {
    std::vector<std::vector<std::pair<Point, line_t>>> points = {
        {{{1, -1}, undefined_lt}, {{1, 0}, undefined_lt}, {{1, 1}, undefined_lt}},
        {{{2, 1}, undefined_lt}, {{2, 2}, undefined_lt}},
        {{{1, 2}, undefined_lt}, {{1, 3}, undefined_lt}}
    };
    EXPECT_FALSE(is_real_size_line_in(points, 1, 0, 1.5));
}

TEST(CoverLines, LeftCover) {
    std::vector<std::vector<std::pair<Point, line_t>>> points = {
        {{{1, -1}, undefined_lt}, {{1, 0}, undefined_lt}, {{1, 1}, undefined_lt}},
        {{{2, 1}, undefined_lt}, {{2, 2}, undefined_lt}},
        {{{3, 2}, undefined_lt}, {{3, 3}, undefined_lt}, {{3, 4}, undefined_lt}}
    };
    EXPECT_FALSE(is_real_size_line_in(points, 1, 0, 1.5));
}

TEST(CoverLines, RightCover) {
    std::vector<std::vector<std::pair<Point, line_t>>> points = {
        {{{3, -1}, undefined_lt}, {{3, 1}, undefined_lt}},
        {{{2, 1}, undefined_lt}, {{2, 2}, undefined_lt}},
        {{{1, 2}, undefined_lt}, {{1, 3}, undefined_lt}}
    };
    EXPECT_FALSE(is_real_size_line_in(points, 1, 0, 1.5));
}

TEST(CoverLines, NotCover) {
    std::vector<std::vector<std::pair<Point, line_t>>> points = {
        {{{3, -1}, undefined_lt}, {{3, 1}, undefined_lt}},
        {{{2, 1}, undefined_lt}, {{2, 2}, undefined_lt}},
        {{{3, 2}, undefined_lt}, {{3, 5}, undefined_lt}}
    };
    EXPECT_TRUE(is_real_size_line_in(points, 1, 0, 1.5));
}


TEST(CoverLines, OneLine) {
    std::vector<std::vector<std::pair<Point, line_t>>> points = {
        {{{1, 1}, undefined_lt}, {{2, 2}, undefined_lt},
         {{3, 2}, undefined_lt}, {{3, 1}, undefined_lt}}
    };
    EXPECT_TRUE(is_real_size_line_in(points, 0, 1, 1.5));
}

TEST(CoverLines, Cube) {
    std::vector<std::vector<std::pair<Point, line_t>>> points = {
        {{{1, -1}, undefined_lt}, {{1, 0}, undefined_lt},
        {{1, 2}, undefined_lt}, {{-1, 2}, undefined_lt},
        {{-1, -1}, undefined_lt}, {{1, -1}, undefined_lt}}
    };
    EXPECT_TRUE(is_real_size_line_in(points, 0, 0, 1.5));
}

TEST(CoverLines, TwoLineUncover) {
    std::vector<std::vector<std::pair<Point, line_t>>> points = {
        {{{-1, 1}, undefined_lt}, {{1, 1}, undefined_lt}},
        {{{1, 2}, undefined_lt},
         {{2, 2}, undefined_lt},
         {{2, -1}, undefined_lt},
         {{-2, -1}, undefined_lt},
         {{-2, 2}, undefined_lt},
         {{-1, 2}, undefined_lt}
        }
    };
    EXPECT_TRUE(is_real_size_line_in(points, 0, 0, 2.5));
}

TEST(CoverLines, TwoLineCover) {
    std::vector<std::vector<std::pair<Point, line_t>>> points = {
        {{{-1, 3}, undefined_lt}, {{1, 3}, undefined_lt}},
        {{{1, 2}, undefined_lt},
         {{2, 2}, undefined_lt},
         {{2, -1}, undefined_lt},
         {{-2, -1}, undefined_lt},
         {{-2, 2}, undefined_lt},
         {{-1, 2}, undefined_lt}
        }
    };
    EXPECT_FALSE(is_real_size_line_in(points, 0, 0, 2.5));
}

TEST(ParallelLine, ArroundPi) {
    EXPECT_TRUE(is_parallel(M_PI - 0.001, -M_PI + 0.001, 0.1));
}

TEST(ParallelLine, Pi4) {
    EXPECT_FALSE(is_parallel(M_PI_4, -M_PI_4, 0.1));
}

TEST(ParallelLine, Pi2) {
    EXPECT_TRUE(is_parallel(M_PI_2, -M_PI_2, 0.1));
}

TEST(ParallelLine, OneSideTrue) {
    EXPECT_TRUE(is_parallel(M_PI_2 + 0.001, M_PI_2, 0.1));
}

TEST(ParallelLine, OneSideFalse) {
    EXPECT_FALSE(is_parallel(M_PI_2, M_PI_4, 0.1));
}

TEST(DetectedType, DetecteBoarder) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MaxCorners_5k.ld", points));
    auto ans = line2line_type(get_corners(points));
    detect_boarder(ans);
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

    const std::pair<int, int> border_ind[] = {{1, 0},
                                              {3, 0},
                                              {3, 1},
                                              {5, 0},
                                              {8, 0},
                                              {8, 1},
                                              {10, 0},
                                              {12, 0},
                                              {12, 1},
                                              {12, 2}};
    int k = 0;
    for (int i = 0; i < ans.size(); i++) {
        for (int j = 0; j < (ans[i].size()); j++) {
            if ((i == border_ind[k].first) && (j == border_ind[k].second)) {
                EXPECT_EQ(ans[i][j].second, border_lt);
                k++;
            } else {
                EXPECT_NE(ans[i][j].second, border_lt);
            }
        }
    }
}

TEST(DetectedType, DetecteParkingZoneRotation1) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("ParkingZone_Rotation1_8k.ld", points));
    auto ans = line2line_type(get_corners(points));
    detect_boarder(ans);
    detected_parking_zone(ans);
    ASSERT_EQ(ans.size(), 4);
    ASSERT_EQ(ans[0].size(), 5);
    ASSERT_EQ(ans[1].size(), 3);
    ASSERT_EQ(ans[2].size(), 3);
    ASSERT_EQ(ans[3].size(), 2);

    const int pz_ind[] = {1, 2};
    int k = 0;
    for (int i = 0; i < ans.size(); i++) {
        for (int j = 0; j < (ans[i].size()); j++) {
            if (i == pz_ind[k]) {
                EXPECT_EQ(ans[i][j].second, parking_lt);
            } else {
                EXPECT_NE(ans[i][j].second, parking_lt);
            }
        }
        if (i == pz_ind[k]) {
            k++;
        }
    }
}

TEST(DetectedType, DetecteParkingZoneRotation0SameLine) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("ParkingZoneAndSameBox_Rotation0_8k.ld", points));
    auto ans = line2line_type(get_corners(points));
    detect_boarder(ans);
    detected_parking_zone(ans);
    ASSERT_EQ(ans.size(), 4);
    ASSERT_EQ(ans[0].size(), 3);
    ASSERT_EQ(ans[1].size(), 4);
    ASSERT_EQ(ans[2].size(), 3);
    ASSERT_EQ(ans[3].size(), 5);

    const int pz_ind[] = {0, 1};
    int k = 0;
    for (int i = 0; i < ans.size(); i++) {
        for (int j = 0; j < (ans[i].size()); j++) {
            if (i == pz_ind[k]) {
                EXPECT_EQ(ans[i][j].second, parking_lt);
            } else {
                EXPECT_NE(ans[i][j].second, parking_lt);
            }
        }
        if (i == pz_ind[k]) {
            k++;
        }
    }
}

TEST(DetectedType, DetecteParkingZoneRotation2SmallWindow) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("ParkingZoneFromSmallWindow_Rotation2_8k.ld", points));
    auto ans = line2line_type(get_corners(points));
    detect_boarder(ans);
    detected_parking_zone(ans);
    ASSERT_EQ(ans.size(), 5);
    ASSERT_EQ(ans[0].size(), 3);
    ASSERT_EQ(ans[1].size(), 2);
    ASSERT_EQ(ans[2].size(), 2);
    ASSERT_EQ(ans[3].size(), 2);
    ASSERT_EQ(ans[4].size(), 4);

    const int pz_ind[] = {1};
    int k = 0;
    for (int i = 0; i < ans.size(); i++) {
        for (int j = 0; j < (ans[i].size()); j++) {
            if (i == pz_ind[k]) {
                EXPECT_EQ(ans[i][j].second, parking_lt);
            } else {
                EXPECT_NE(ans[i][j].second, parking_lt);
            }
        }
        if (i == pz_ind[k]) {
            k++;
        }
    }
}
