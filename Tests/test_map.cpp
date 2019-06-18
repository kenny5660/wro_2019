//
// Created by Danila on 29.03.2019.
//

#include "test.h"
#include "..\highLogic\map.h"
#include "../highLogic/CV.h"
#include <string>

TEST(BoxMap, Create) {
    {
        BoxMap m1;
        EXPECT_EQ(m1.get_left_corner_point().get_x(), 0);
        EXPECT_EQ(m1.get_left_corner_point().get_y(), 0);
        EXPECT_EQ(m1.get_color(), undefined_c);
    }
    {
        BoxMap m1(Point(1, 1));
        EXPECT_EQ(m1.get_left_corner_point().get_x(), 1);
        EXPECT_EQ(m1.get_left_corner_point().get_y(), 1);
        EXPECT_EQ(m1.get_color(), undefined_c);
    }
}

TEST(BoxMap, SetColor) {
    {
        BoxMap m1(Point(1, 1), red_c);
        EXPECT_EQ(m1.get_color(), red_c);
    }
    {
        BoxMap m1;
        m1.set_color(blue_c);
        EXPECT_EQ(m1.get_color(), blue_c);
        m1.set_color(red_c);
        EXPECT_EQ(m1.get_color(), blue_c);
    }
}

TEST(BoxMap, GetPoint) {
    BoxMap m1;
    m1.add_point(Point(2, 4));
    EXPECT_NEAR(m1.get_left_corner_point().get_x(), 2, PRECISION_LENGTH);
    EXPECT_NEAR(m1.get_left_corner_point().get_y(), 4, PRECISION_LENGTH);
    m1.add_point(Point(5, 6));
    m1.add_point(Point(2, 7));
    m1.add_point(Point(1, 3));
    EXPECT_NEAR(m1.get_left_corner_point().get_x(), 2.5, PRECISION_LENGTH);
    EXPECT_NEAR(m1.get_left_corner_point().get_y(), 5, PRECISION_LENGTH);
    auto corners = m1.get_corners();
    EXPECT_NEAR(corners[0].get_x(), 2.5, PRECISION_LENGTH);
    EXPECT_NEAR(corners[0].get_y(), 5, PRECISION_LENGTH);
    EXPECT_NEAR(corners[1].get_x(), 2.5 + field_sett::climate_box_width, PRECISION_LENGTH);
    EXPECT_NEAR(corners[1].get_y(), 5, PRECISION_LENGTH);
    EXPECT_NEAR(corners[2].get_x(), 2.5 + field_sett::climate_box_width, PRECISION_LENGTH);
    EXPECT_NEAR(corners[2].get_y(), 5 + field_sett::climate_box_height, PRECISION_LENGTH);
    EXPECT_NEAR(corners[3].get_x(), 2.5, PRECISION_LENGTH);
    EXPECT_NEAR(corners[3].get_y(), 5 + field_sett::climate_box_height, PRECISION_LENGTH);
}

TEST(BoxMap, getBoxOffsetRight) {
    BoxMap b(Point{field_sett::size_field_unit * 3, field_sett::size_field_unit * 10});
    Point p = b.get_box_indent();
    EXPECT_NEAR(p.get_x(), field_sett::size_field_unit * 7, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), field_sett::size_field_unit * 11, PRECISION_LENGTH);
}

TEST(BoxMap, getBoxOffsetLeft) {
    BoxMap b(Point{field_sett::size_field_unit * 17, field_sett::size_field_unit * 10});
    Point p = b.get_box_indent();
    EXPECT_NEAR(p.get_x(), field_sett::size_field_unit * 15, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), field_sett::size_field_unit * 11, PRECISION_LENGTH);
}

TEST(BoxMap, getBoxOffsetDown) {
    BoxMap b(Point{field_sett::size_field_unit * 7, field_sett::size_field_unit * 1});
    Point p = b.get_box_indent();
    EXPECT_NEAR(p.get_x(), field_sett::size_field_unit * 8, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), field_sett::size_field_unit * 5, PRECISION_LENGTH);
}

TEST(BoxMap, getBoxOffsetUp) {
    BoxMap b(Point{field_sett::size_field_unit * 5, field_sett::size_field_unit * 17});
    Point p = b.get_box_indent();
    EXPECT_NEAR(p.get_x(), field_sett::size_field_unit * 6, PRECISION_LENGTH);
    EXPECT_NEAR(p.get_y(), field_sett::size_field_unit * 15, PRECISION_LENGTH);
}

std::string get_death_zone(const std::array<std::array<bool, field_sett::number_field_unit>, field_sett::number_field_unit> &a) {
    std::string s;
    for (int i = 0; i < a.size(); i++) {
        for (int j = 0; j < a[i].size(); j++) {
            s += std::to_string(a[i][j]) + " ";
        }
        s += "\n";
    }
    std::cout << s << std::endl;
    return s;
}

TEST(MapCreateByPoints, Angle0) {
    Map m({1200, 1200}, {1200, 1200 + field_sett::parking_zone_door_size});
    //show_debug_img("", m.get_img(400, 400));
    auto pz = m.get_parking_zone();
    EXPECT_NEAR(pz[0].get_x(), 1200, PRECISION_LENGTH);
    EXPECT_NEAR(pz[0].get_y(), 1200, PRECISION_LENGTH);
    EXPECT_NEAR(pz[1].get_x(), 1200, PRECISION_LENGTH);
    EXPECT_NEAR(pz[1].get_y(), 1200 + field_sett::parking_zone_door_size, PRECISION_LENGTH);
    EXPECT_NEAR(pz[2].get_x(), 1200 + field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_NEAR(pz[2].get_y(), 1200, PRECISION_LENGTH);
    EXPECT_NEAR(pz[3].get_x(), 1200 + field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_NEAR(pz[3].get_y(), 1200 + field_sett::parking_zone_door_size, PRECISION_LENGTH);
    EXPECT_EQ(get_death_zone(m.get_death_zone()), get_death_zone({{
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}}}));
}

TEST(MapCreateByPoints, Angle6) {
    // TODO:
    Map m({962, 1199}, {961 - 328.114498122, 1202 + 106.610863059});
    //show_debug_img("", m.get_img(400, 400));
    auto pz = m.get_parking_zone();
    EXPECT_NEAR(pz[0].get_x(), 960, PRECISION_LENGTH);
    EXPECT_NEAR(pz[0].get_y(), 1200, PRECISION_LENGTH);
    EXPECT_NEAR(pz[1].get_x(), 960 - sin(atan(3)) * field_sett::parking_zone_door_size, PRECISION_LENGTH);
    EXPECT_NEAR(pz[1].get_y(), 1200 + cos(atan(3)) * field_sett::parking_zone_door_size, PRECISION_LENGTH);
    EXPECT_NEAR(pz[2].get_x(), 960 + cos(atan(3)) * field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_NEAR(pz[2].get_y(), 1200 - sin(atan(3)) * field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_NEAR(pz[3].get_x(), 960 - sin(atan(3)) * field_sett::parking_zone_door_size + cos(atan(3)) * field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_NEAR(pz[3].get_y(), 1200 + cos(atan(3)) * field_sett::parking_zone_door_size - sin(atan(3)) * field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_EQ(get_death_zone(m.get_death_zone()), get_death_zone({{
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}}}));
}

TEST(MapCreateByPoints, Angle_2) {
    Map m({960, 1202}, {961 + sin(atan(1 / 2.)) * field_sett::parking_zone_door_size,
                        1202 + cos(atan(1 / 2.)) * field_sett::parking_zone_door_size});
    //show_debug_img("", m.get_img(400, 400));
    auto pz = m.get_parking_zone();
    EXPECT_NEAR(pz[0].get_x(), 960, PRECISION_LENGTH);
    EXPECT_NEAR(pz[0].get_y(), 1200, PRECISION_LENGTH);
    EXPECT_NEAR(pz[1].get_x(), 960 + sin(atan(1 / 2.)) * field_sett::parking_zone_door_size, PRECISION_LENGTH);
    EXPECT_NEAR(pz[1].get_y(), 1200 + cos(atan(1 / 2.)) * field_sett::parking_zone_door_size, PRECISION_LENGTH);
    EXPECT_NEAR(pz[2].get_x(), 960 + cos(atan(1 / 2.)) * field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_NEAR(pz[2].get_y(), 1200 + sin(atan(1 / 2.)) * field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_NEAR(pz[3].get_x(), 960 + sin(atan(1 / 2.)) * field_sett::parking_zone_door_size + cos(atan(1 / 2.)) * field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_NEAR(pz[3].get_y(), 1200 + cos(atan(1 / 2.)) * field_sett::parking_zone_door_size + sin(atan(1 / 2.)) * field_sett::parking_zone_width_min, PRECISION_LENGTH);
    EXPECT_EQ(get_death_zone(m.get_death_zone()), get_death_zone({{
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}}}));
}

bool boxes_exist(const std::vector<Point> &boxes, const Point &p) {
    const double accuracy = 0.001;
    for (auto i : boxes) {
        if (((fabs(i.get_x() - p.get_x())) <= accuracy) && ((fabs(i.get_y() - p.get_y())) <= accuracy)) {
            return true;
        }
    }
    return false;
}

bool boxes_exist(const std::vector<Point> &boxes, const std::vector<Point> &p) {
    if (boxes.size() != p.size()) {
        return false;
    }
    bool ans = true;
    for (auto i : p) {
        ans = ans && boxes_exist(boxes, i);
    }
    return ans;
}

TEST(Map, CreatMapMaxCorner) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MaxCorners_5k.ld", points));
    auto lines = detected_lines_and_types(points);
    Map m(lines);
    show_debug_img("", m.get_img(400, 400));
    RobotPoint position = m.get_position();
    EXPECT_NEAR(position.get_x(), 1220, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_y(), 2155, PRECISION_LENGTH_POS);
    //EXPECT_NEAR(position.get_angle(), 0.785398, PRECISION_ANGLE_POS);
    EXPECT_EQ(lines2string(m.get_parking_zone_line()),
        "{{(-33.175000, 1681.581000), (-18.313000, 1675.561000), (135.304000, 1979.765000), (355.106000, 1870.454000)}, {(290.355000, 1518.959000), (307.537000, 1515.684000), (461.790000, 1824.203000)}}");
    EXPECT_TRUE(boxes_exist(m.get_boxes_normal(),
        {{1 * field_sett::size_field_unit, 10 * field_sett::size_field_unit},
         {1 * field_sett::size_field_unit, 15 * field_sett::size_field_unit},
         {6 * field_sett::size_field_unit, 4 * field_sett::size_field_unit},
         {17 * field_sett::size_field_unit, 10 * field_sett::size_field_unit},
         {16 * field_sett::size_field_unit, 15 * field_sett::size_field_unit}}));
    EXPECT_EQ(get_death_zone(m.get_death_zone()), get_death_zone({{
              {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0},
              {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0}
    }}));
}

TEST(Map, CreatMapMaxCornerFreeField) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("FreeField_200_300_23_8k.ld", points));
    auto lines = detected_lines_and_types(points);
    Map m(lines, show_debug_img);
    show_debug_img("", m.get_img(400, 400));
    RobotPoint position = m.get_position();
    EXPECT_NEAR(position.get_x(), 200, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_y(), 300, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_angle(), degree2radian(23), PRECISION_ANGLE_POS);
    EXPECT_EQ(lines2string(m.get_parking_zone_line()),
        "{{(1015.319000, -899.465000), (999.566000, -905.318000), (999.500000, -1243.683000), (905.032000, -1243.730000)}, {(654.164000, -900.463000), (636.570000, -901.309000), (636.633000, -1256.912000)}}");
    EXPECT_TRUE(boxes_exist(m.get_boxes_normal(), std::vector<Point>()));
    EXPECT_EQ(get_death_zone(m.get_death_zone()), get_death_zone({{
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1}
              }}));
}

TEST(Map, CreatMapMaxBoxAndMargTogether) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot2/1.ld", points));
    auto lines = detected_lines_and_types(points);
    Map m(lines, show_debug_img);
    show_debug_img("", m.get_img(400, 400));
    RobotPoint position = m.get_position();
    EXPECT_NEAR(position.get_x(), 1900, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_y(), 200, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_angle(), degree2radian(290), PRECISION_ANGLE_POS);
    EXPECT_EQ(lines2string(m.get_parking_zone_line()),
              "{{(-1059.553000, -653.359000), (-1071.580000, -636.624000), (-1417.891000, -636.914000)}}");
    EXPECT_TRUE(boxes_exist(m.get_boxes_normal(),
                            {{1 * field_sett::size_field_unit, 3 * field_sett::size_field_unit},
                             {18 * field_sett::size_field_unit, 5 * field_sett::size_field_unit},
                             {18 * field_sett::size_field_unit, 10 * field_sett::size_field_unit},
                             {12 * field_sett::size_field_unit, 5 * field_sett::size_field_unit},
                             {12 * field_sett::size_field_unit, 11 * field_sett::size_field_unit}}));
    EXPECT_EQ(get_death_zone(m.get_death_zone()), get_death_zone({{
                  {0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0}
              }}));
}

TEST(Map, CreatMapMax2BoxesAndMargTogether) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot3/0.ld", points));
    auto lines = detected_lines_and_types(points);
    Map m(lines, show_debug_img);
    show_debug_img("", m.get_img(400, 400));
    RobotPoint position = m.get_position();
    EXPECT_NEAR(position.get_x(), 1248, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_y(), 1352, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_angle(), degree2radian(0), PRECISION_ANGLE_POS);
    EXPECT_EQ(lines2string(m.get_parking_zone_line()),
              "{{(-210.335000, 548.285000), (110.754000, 386.198000), (278.758000, 719.543000)}}");
    EXPECT_TRUE(boxes_exist(m.get_boxes_normal(),
                            {{0 * field_sett::size_field_unit, 6 * field_sett::size_field_unit},
                             {18 * field_sett::size_field_unit, 6 * field_sett::size_field_unit},
                             {4 * field_sett::size_field_unit, 14 * field_sett::size_field_unit},
                             {15 * field_sett::size_field_unit, 14 * field_sett::size_field_unit},
                             {9 * field_sett::size_field_unit, 18 * field_sett::size_field_unit}}));
    EXPECT_EQ(get_death_zone(m.get_death_zone()), get_death_zone({{
                  {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
                  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
                  {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1}
              }}));
}

TEST(Map, MergeRot0) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot0/0.ld", points));
    Map m1(points);
    points.clear();
    show_debug_img("m1", m1.get_img());
    ASSERT_FALSE(read("MergeMapRot0/1.ld", points));
    Map m2(points);
    show_debug_img("m2", m2.get_img());
    m1.merge(m2);
    auto position = m1.get_position();
    auto expected_position = m2.get_position();
    EXPECT_NEAR(position.get_x(), expected_position.get_x(), PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_y(), expected_position.get_y(), PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_angle(), expected_position.get_angle(), PRECISION_ANGLE_POS);
    EXPECT_TRUE(boxes_exist(m1.get_boxes_normal(),
                            {{2 * field_sett::size_field_unit, 3 * field_sett::size_field_unit},
                             {2 * field_sett::size_field_unit, 13 * field_sett::size_field_unit},
                             {2 * field_sett::size_field_unit, 16 * field_sett::size_field_unit},
                             {15 * field_sett::size_field_unit, 3 * field_sett::size_field_unit},
                             {15 * field_sett::size_field_unit, 11 * field_sett::size_field_unit}}));
    EXPECT_EQ(get_death_zone(m1.get_death_zone()), get_death_zone({{
                  {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0},
                  {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0},
                  {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
                  {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                  {1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                  {1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0}
              }}));
}

TEST(Map, MergeRot1) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot1/0.ld", points));
    Map m1(points);
    points.clear();
    show_debug_img("m1", m1.get_img());
    ASSERT_FALSE(read("MergeMapRot1/1.ld", points));
    Map m2(points);
    show_debug_img("m2", m2.get_img());
    m1.merge(m2);
    auto position = m1.get_position();
    EXPECT_NEAR(position.get_x(), 900, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_y(), 200, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_angle(), degree2radian(150), PRECISION_ANGLE_POS);
    EXPECT_TRUE(boxes_exist(m1.get_boxes_normal(),
                            {{1 * field_sett::size_field_unit, 8 * field_sett::size_field_unit},
                             {1 * field_sett::size_field_unit, 10 * field_sett::size_field_unit},
                             {1 * field_sett::size_field_unit, 12 * field_sett::size_field_unit},
                             {1 * field_sett::size_field_unit, 14 * field_sett::size_field_unit},
                             {1 * field_sett::size_field_unit, 16 * field_sett::size_field_unit}}));
    EXPECT_EQ(get_death_zone(m1.get_death_zone()), get_death_zone({{
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
               }}));
}

TEST(Map, MergeRot2) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot2/0.ld", points));
    Map m1(points);
    points.clear();
    show_debug_img("m1", m1.get_img());
    ASSERT_FALSE(read("MergeMapRot2/1.ld", points));
    Map m2(points);
    show_debug_img("m2", m2.get_img());
    m1.merge(m2);
    auto position = m1.get_position();
    EXPECT_NEAR(position.get_x(), 200, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_y(), 500, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_angle(), degree2radian(200), PRECISION_ANGLE_POS);
    EXPECT_TRUE(boxes_exist(m1.get_boxes_normal(),
                            {{5 * field_sett::size_field_unit, 0 * field_sett::size_field_unit},
                             {10 * field_sett::size_field_unit, 0 * field_sett::size_field_unit},
                             {5 * field_sett::size_field_unit, 6 * field_sett::size_field_unit},
                             {11 * field_sett::size_field_unit, 6 * field_sett::size_field_unit},
                             {3 * field_sett::size_field_unit, 17 * field_sett::size_field_unit}}));
    EXPECT_EQ(get_death_zone(m1.get_death_zone()), get_death_zone({{
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0},
                   {1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0},
                   {1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
                   {1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
                   {1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                   {1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
               }}));
}

TEST(Map, MergeRot3) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot3/0.ld", points));
    Map m1(points);
    points.clear();
    show_debug_img("m1", m1.get_img());
    ASSERT_FALSE(read("MergeMapRot3/1.ld", points));
    Map m2(points);
    show_debug_img("m2", m2.get_img());
    m1.merge(m2);
    auto position = m1.get_position();
    EXPECT_NEAR(position.get_x(), 1200, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_y(), 1300, PRECISION_LENGTH_POS);
    EXPECT_NEAR(position.get_angle(), degree2radian(300), PRECISION_ANGLE_POS);
    EXPECT_TRUE(boxes_exist(m1.get_boxes_normal(),
                            {{0 * field_sett::size_field_unit, 6 * field_sett::size_field_unit},
                             {18 * field_sett::size_field_unit, 6 * field_sett::size_field_unit},
                             {4 * field_sett::size_field_unit, 14 * field_sett::size_field_unit},
                             {15 * field_sett::size_field_unit, 14 * field_sett::size_field_unit},
                             {9 * field_sett::size_field_unit, 18 * field_sett::size_field_unit}}));
    EXPECT_EQ(get_death_zone(m1.get_death_zone()), get_death_zone({{
                   {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
                   {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1}
               }}));
}

TEST(Map, NormalDeathZone) {
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("MergeMapRot2/1.ld", points));
    Map m(points);
    show_debug_img("", m.get_img());
    // TODO: учесть свободное пространство перед квадратом. Скрыть данную функцию. Добавить с учётом кводратов.
    m.normal_death_zone();
    show_debug_img("", m.get_img());
}

TEST(MapCreateBorder, Merge) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(N,G,O,I)(Q,Q,O,S)(F,G,D,I)(U,L,S,N)");
    start_position.set_angle(0);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("", img);
}

TEST(MapCreateBorder, Merge2) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(H,K,K,K)(A,A,C,C)(I,A,K,C)(E,A,G,C)");
    start_position.set_angle(0);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("", img);
}


TEST(MapCreateBorder, NonMerge) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(N,G,O,I)(Q,Q,O,S)(F,G,D,I)(A,R,C,T)");
    start_position.set_angle(0);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("", img);
}

TEST(MapCreateBorder, r) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz; // TODO:
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(C,C,G,C)(H,H,J,J)(Q,Q,O,S)(A,R,C,T)");
    start_position.set_angle(0);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("", img);
}

TEST(MapUpdate, up1) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position = qr_detect(QRCodeImg, boxes, pz, "(N,G,O,I)(Q,Q,O,S)(F,G,D,I)(U,L,S,N)");
    start_position.set_angle(0);
    start_position.set_x(1 * field_sett::size_field_unit);
    start_position.set_y(8 * field_sett::size_field_unit);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("HalfData//(NGOI)(QQOS)(FGDI)(ULSN).ld", points));
    map.update(points, show_debug_img);

//    std::vector<std::vector<Point>> points;
//    std::pair<Point, Point> parking_zone_circles_ = {
//        {123, 456},
//        {456, 321}
//    };
//    std::pair<double , double> p(250, 350);
//    std::vector<std::vector<std::pair<Point, line_t>>> lines;
//    line_detect_from_pos(lines, parking_zone_circles_,
//                         points, p, {0, 0});
}

TEST(MapUpdate, up2) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(0);
    start_position.set_x(400);
    start_position.set_y(1100);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("HalfData//(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)_from_frame_root0.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, up3) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(0);
    start_position.set_x(900);
    start_position.set_y(666);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("HalfData//(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)_1_root0.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, up3WithRoot90) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(degree2radian(90));
    start_position.set_x(900);
    start_position.set_y(666);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("HalfData//(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)_1_root90.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, up3WithRoot100) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(degree2radian(110));
    start_position.set_x(900);
    start_position.set_y(666);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("HalfData//(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)_1_root100.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, RealUp) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(2.6853440000000002);
    start_position.set_x(1162.1408730350097);
    start_position.set_y(1266.5040220418102);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//1.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, RealUp2) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(2.6853440000000002);
    start_position.set_x(1162.1408730350097);
    start_position.set_y(1266.5040220418102);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//2.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, RealUp3) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(2.6853440000000002);
    start_position.set_x(1162.1408730350097);
    start_position.set_y(1266.5040220418102);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//3.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, RealUp4) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(2.6853440000000002);
    start_position.set_x(1162.1408730350097);
    start_position.set_y(1266.5040220418102);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//4.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, RealUp5) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(2.6853440000000002);
    start_position.set_x(1162.1408730350097);
    start_position.set_y(1266.5040220418102);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//5.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, RealUp6) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    //RobotPoint start_position =
    //    qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    //start_position.set_angle(2.6853440000000002);
    //start_position.set_x(1162.1408730350097);
    //start_position.set_y(1266.5040220418102);
    //Map map(pz.first, pz.second, boxes, start_position);
//    cv::Mat img = map.get_img();
//    show_debug_img("Map", img);
//    std::vector<PolarPoint> points;
//    ASSERT_FALSE(read("Real//5.ld", points));
//    map.update(points, show_debug_img);
}

TEST(MapUpdate, RealUp7) {
    Map map(RobotPoint{382.08470086484556, 1204.1152715488881, 5.7522159999999998}, Point{553.43096886860883, 903.45482113815888}, Point{728.12842176093397, 1200.9540647555737});
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;

    ASSERT_FALSE(read("Real//9.ld", points));
    map.update(points, show_debug_img);
}

TEST(MapUpdate, RealUp10) {
    cv::Mat QRCodeImg;
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint start_position =
        qr_detect(QRCodeImg, boxes, pz, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
    start_position.set_angle(5.7522159999999998);
    start_position.set_x(382.08470086484556);
    start_position.set_y(1204.1152715488881);
    Map map(pz.first, pz.second, boxes, start_position);
    cv::Mat img = map.get_img();
    show_debug_img("Map", img);
    std::vector<PolarPoint> points;
    ASSERT_FALSE(read("Real//9.ld", points));
    map.update(points, show_debug_img);
}
