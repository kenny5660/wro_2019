//
// Created by Danila on 18.03.2019.
//

#include "../test.h"

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
