//
// Created by Danila on 14.02.2019.
//

#include "logic_structures.h"

double angle_normalization(double angle) {
    if(std::isnan(angle))
        return NAN;
    double (*fmod)(double, double) = [](double a, double b) {
      const long long prec = 1e6;
      return ((double)((long long)(a * prec) % (long long)(b * prec)))
          / prec;
    };
    return fmod((fmod(angle, DOUBLE_PI) + DOUBLE_PI), DOUBLE_PI);
}

PolarPoint Point::to_polar() {
    double dist = sqrt(x_ * x_ + y_ * y_);
    return PolarPoint(dist, (dist >= 0.0000001) ? (atan2(y_, x_)) : 0);
}

Point &Point::operator += (const Point &a) {
    x_ += a.x_;
    y_ += a.y_;
    return *this;
}

Point &Point::operator -= (const Point &a) {
    x_ -= a.x_;
    y_ -= a.y_;
    return *this;
}

RobotPoint::RobotPoint(double x, double y, double angle) {
    x_ = x;
    y_ = y;
    set_angle(angle);
}

void RobotPoint::set_angle(double angle) {
    angle_ = angle_normalization(angle);
}

void RobotPoint::merge(const RobotPoint &a) {
    double_merge(x_, a.x_);
    double_merge(y_, a.y_);
    double_merge(angle_, a.angle_);
}

void RobotPoint::double_merge(double &a, const double b) {
    if (std::isnan(a)) {
        a = b;
    } else {
        if (std::isnan(b))
            return;
        a = (a + b) / 2;
    }
}

void PolarPoint::set_f(double f) {
    f_ = angle_normalization(f);
};
