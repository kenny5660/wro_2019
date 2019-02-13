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
    double get_x() { return x_; }
    double get_y() { return  y_; }

    void set_x(double x) { x_ = x; }
    void set_y(double y) { y_ = y; }

 protected:
    double x_;
    double y_;
};

class RobotPoint: public Point {
 public:
    RobotPoint(double x = 0, double y = 0, double angle = 0);

    double get_angle() { return angle_; }
    void set_angle(double angle);
    void add_angle(double add) { set_angle(angle_ + add); }

    void merge(const RobotPoint &a);

 protected:
    double angle_;

 private:
    static void double_merge(double &a, const double b);
};

#endif //LIDAR_MATH_LOGIC_STRUCTURES_H
