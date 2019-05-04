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

void Point::rotation(double ang) { // вращение по часовой стрелке
    double X = x_ * cos(ang) - y_ * sin(ang);
    double Y = x_ * sin(ang) + y_ * cos(ang);
    x_ = X;
    y_ = Y;
}

MassPoint::MassPoint(double x, double y) {
    x_ = x;
    y_ = y;
    if (!std::isnan(x_)) {
        count_x_ = 1;
    }
    if (!std::isnan(y_)) {
        count_y_ = 1;
    }
}

MassPoint::MassPoint(const Point &p) {
    x_ = p.get_x();
    y_ = p.get_y();
    if (!std::isnan(x_)) {
        count_x_ = 1;
    }
    if (!std::isnan(y_)) {
        count_y_ = 1;
    }
}

void MassPoint::set_x(double x) {
    Point::set_x(x);
    count_x_++;
}

void MassPoint::set_y(double y) {
    Point::set_y(y);
    count_y_++;
}

void MassPoint::count_merge(const MassPoint &p) {
    count_x_ = std::min(count_x_, p.count_x_);
    count_y_ = std::min(count_y_, p.count_y_);
}

MassPoint MassPoint::operator + (const MassPoint &a) const {
    MassPoint ans(x_ + a.x_, y_ + a.y_);
    ans.count_merge(a);
    return ans;
}

MassPoint MassPoint::operator - (const MassPoint &a) const {
    MassPoint ans(x_ - a.x_, y_ - a.y_);
    ans.count_merge(a);
    return ans;
}

MassPoint &MassPoint::operator += (const MassPoint &a) {
    x_ += a.x_;
    y_ += a.y_;
    count_merge(a);
    return *this;
}

MassPoint &MassPoint::operator -= (const MassPoint &a) {
    x_ -= a.x_;
    y_ -= a.y_;
    count_merge(a);
    return *this;
}

void MassPoint::double_merge(double &a, const double b, size_t &counter_a, const size_t counter_b) {
    if (std::isnan(a)) {
        a = b;
        counter_a = counter_b;
    } else {
        if (std::isnan(b)) {
            return;
        }
        a = (a * counter_a + b * counter_b) / (counter_a + counter_b);
        counter_a += counter_b;
    }
}

void MassPoint::set_mass(std::pair<size_t , size_t> m) {
    count_x_ = m.first;
    count_y_ = m.second;
}

void MassPoint::merge(const MassPoint &p) {
    double_merge(x_, p.x_, count_x_, p.count_x_);
    double_merge(y_, p.y_, count_y_, p.count_y_);
}

RobotPoint::RobotPoint(double x, double y, double angle) {
    x_ = x;
    if (!std::isnan(x)) {
        count_x_ = 1;
    }
    y_ = y;
    if (!std::isnan(y)) {
        count_y_ = 1;
    }
    set_angle(angle);
    if (!std::isnan(angle)) {
        count_angle_ = 1;
    }
}

void RobotPoint::set_angle(double angle) {
    angle_ = angle_normalization(angle);
    count_angle_ = 1;
}

void RobotPoint::merge(const RobotPoint &a) {
    MassPoint::merge(a);
    double_merge(angle_, a.angle_, count_angle_, a.count_angle_);
}

void PolarPoint::set_f(double f) {
    f_ = angle_normalization(f);
};

double PolarPoint::angle_norm(double a) {
    return angle_normalization(a);
}