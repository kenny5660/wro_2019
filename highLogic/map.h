//
// Created by Danila on 28.03.2019.
//

#ifndef LIDAR_MATH_MAP_H
#define LIDAR_MATH_MAP_H

#include <array>
#include <list>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "logic_structures.h"
#include "debug.h"
#include "settings.h"

enum box_color_t {
    undefined_bc,
    blue_bc,
    red_bc,
    green_bc,
    orange_bc,
    yellow_bc
};

const cv::Scalar box_color2color[6] = {
    {218, 197, 255},
    {159, 6, 16},
    {55, 39, 238},
    {44, 214, 68},
    {0, 94, 254},
    {0, 233, 254}};

class BoxMap {
 public:
    BoxMap() : left_up_corner_(Point(0, 0)) {};
    BoxMap(const Point &p, box_color_t color = undefined_bc);

    void add_point(const Point&);
    MassPoint set_left_corner_point(const MassPoint &p) { left_up_corner_ = p; }
    MassPoint get_left_corner_point() const { return left_up_corner_; }
    MassPoint add_left_corner_point(const MassPoint &p) { left_up_corner_.merge(p); }
    MassPoint add_left_corner_point(const Point &p) { left_up_corner_.merge(MassPoint(p)); }

    void merge(const BoxMap &);

    Point get_box_indent();

    std::array<Point, 4> get_corners() const;

    void set_color(const box_color_t);
    box_color_t get_color() { return color_; }

 private:
    MassPoint left_up_corner_;
    box_color_t color_ = undefined_bc;
};

class Map {
 public:
    Map(const Point &p1, const Point &p2);
    Map(const std::vector<std::vector<std::pair<Point, line_t>>> &, show_img_debug debug = nullptr);
    Map(const std::vector<PolarPoint> &, show_img_debug debug = nullptr);
    Map(const Point &p_1, const Point &p_2, const std::array<BoxMap, 3> &boxes, const RobotPoint &p);

    void normal_death_zone();
    bool merge(const Map &m);
    void norm_merge(const Map &m);


    std::array<Point, 4> get_parking_zone() const;
    std::vector<std::vector<Point>> get_parking_zone_line() const;
    std::vector<Point> get_boxes_normal() const;
    std::array<std::array<bool, field_sett::number_field_unit>,
               field_sett::number_field_unit> get_death_zone() const;
    RobotPoint get_position() const { return position_; }
    cv::Mat get_img(int width = 400, int height = 400);

    void set_new_position(const RobotPoint &p) { position_ = p; }

    static Point normal_point(const Point &p);
    static std::pair <int, int> get_field_unit(const Point &p);

    void update(const std::vector<PolarPoint> &);

 private:
    bool add_box(const Point &p);
    void add_boxes_in_robot_pos(const Point &corn, const Point &next);
    void delete_from_death_zone_circle(const Point &p, double r = field_sett::parking_zone_free_radius);
    void add_death_outline(const std::vector<Point> &outline);
    void lines_detection(const std::vector<std::vector<std::pair<Point, line_t>>> &, show_img_debug debug = nullptr);

    std::pair<int, double> get_nearby_box(const Point &p);
    void turn(int = 1);

    bool in_death_zone(const Point &);
    bool death_rect(const cv::Point2i &a, const cv::Point2i &b);

    void add_pz(const Point &p_1, const Point &p_2);

    std::pair<Point, Point> parking_zone_circles_ = std::make_pair(Point(), Point()); // главня точка - первая
    std::pair<Point, Point> parking_zone_back_; // ниже главной - первая
    std::vector<std::vector<Point>> parking_detected_line_;
    std::array<BoxMap, 5> boxes_;
    size_t box_count_ = 0;
    RobotPoint position_;
    std::array<std::array<bool, field_sett::number_field_unit>, field_sett::number_field_unit> death_zone_;
};

#endif //LIDAR_MATH_MAP_H
