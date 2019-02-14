//
// Created by Danila on 14.02.2019.
//

#ifndef LIDAR_MATH_LOGIC_STRUCTURES_H
#define LIDAR_MATH_LOGIC_STRUCTURES_H

#include <math.h>
#define DOUBLE_PI (2 * M_PI)

enum field_margin {
    left_field_margin,
    top_field_margin,
    right_field_margin,
    bottom_field_margin
};

enum field_corner {
    top_left_corner,
    top_right_corner,
    bottom_right_corner,
    bottom_left_corner
};

class Point {
 public:
    Point(double x = NAN, double y = NAN): x_(x), y_(y) {}

    double get_x() { return x_; }
    double get_y() { return  y_; }

    void set_x(double x) { x_ = x; }
    void set_y(double y) { y_ = y; }

    Point operator + (const Point &a) { return Point(x_ + a.x_, y_ + a.y_); }

 protected:
    double x_;
    double y_;
};

class PolarPoint {
 public:
    PolarPoint(double r = NAN, double f = NAN): r_(r) { set_f(f); }

    double get_r() const { return r_; }
    double get_f() const { return  f_; }

    void set_r(double r) { r_ = r; }
    void set_f(double f);

    void add_f(double f) { set_f(f_ + f); }

    Point to_cartesian() const {
        return Point(r_ * cos(f_), r_ * sin(f_));
    }

 protected:
    double r_;
    double f_;
};

class RobotPoint: public Point {
 public:
    RobotPoint(double x = NAN, double y = NAN, double angle = NAN);

    double get_angle() const { return angle_; }
    void set_angle(double angle);
    void add_angle(double add) { set_angle(angle_ + add); }

    void merge(const RobotPoint &a);

 protected:
    double angle_;

 private:
    static void double_merge(double &a, const double b);
};

#endif //LIDAR_MATH_LOGIC_STRUCTURES_H
