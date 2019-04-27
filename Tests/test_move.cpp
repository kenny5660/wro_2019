//
// Created by Danila on 20.04.2019.
//

#include "test.h"

TEST(Move, MakeWayFree) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot3/0.ld", points));
    Map m1(points);
    std::vector<Point> way;
    EXPECT_TRUE(go_to(m1, {field_sett::max_field_width - 120, 120}, way));
    EXPECT_FALSE(way.empty());
}

TEST(Move, MakeWayDeath) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot3/0.ld", points));
    Map m1(points);
    //save_debug_img("", m1.get_img());
    std::vector<Point> way;
    EXPECT_FALSE(go_to(m1, {1000, 500}, way));
    EXPECT_FALSE(way.empty());
}

TEST(Move, addPZ) {
    //зайти ручками и проверить, что на нужном месте позиция занята
    Map m1({480, 120}, {480, 120 + field_sett::parking_zone_door_size});
    std::vector<Point> way;
    go_to(m1, {1000, 500}, way);
}
