//
// Created by Danila on 29.01.2019.
//

#ifndef LIDAR_MATH_LIDAR_MATH_H
#define LIDAR_MATH_LIDAR_MATH_H

#include <math.h>

#define DOUBLE_PI (2 * M_PI)

#define MAX_FIELD_WIDTH (24000)
#define MAX_FIELD_HEIGHT MAX_FIELD_WIDTH

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

class RobotPoint {
 public:
    RobotPoint(double x = 0, double y = 0, double angle = 0) : x_(x), y_(y) {
        set_angle(angle);
    }

    double get_x() { return x_; }
    double get_y() { return y_; }
    double get_angle() { return angle_; }

    void set_x(double x) { x_ = x; }
    void set_y(double y) {y_ = y; }
    void set_angle(double angle) {
        double (*fmod)(double, double) = [](double a, double b) {
          const long long prec = 1e6;
          return ((double)((long long)(a * prec) % (long long)(b * prec)))
              / prec;
        };
        angle_ = fmod((fmod(angle, DOUBLE_PI) + DOUBLE_PI), DOUBLE_PI);
    };

    void merge(const RobotPoint &a) {
        double_merge(x_, a.x_);
        double_merge(y_, a.y_);
        double_merge(angle_, a.angle_);
    }

    void add_angle(double add) { set_angle(angle_ + add); }

 private:
    double x_;
    double y_;
    double angle_;

    static void double_merge(double &a, const double b) {
        if (isnan(a)) {
            a = b;
        } else {
            a = (a + b) / 2;
        }
    }
};

RobotPoint init_position_from_line(double b, double c, double alpha);
RobotPoint get_coordinates_from_line(double b, double c, double alpha,
                                     double b_angle_offset,
                                     field_margin margin);

void location_recognition_model_test();

#endif //LIDAR_MATH_LIDAR_MATH_H
