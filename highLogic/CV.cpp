//
// Created by Danila on 06.05.2019.
//

#include "CV.h"
#include <iostream>
#include "settings.h"
#include "debug.h"

inline double letter2coordinat(const char a) {
    return (a - 'A') * field_sett::size_field_unit;
}

inline Point letter2coordinat(const char a, const char b) {
    return Point{letter2coordinat(a), letter2coordinat(b)};
}

Point (*rot2point[4])(Point &p) = {
    [](Point &p) {
      return p;
    },
    [](Point &p) {
      return Point{field_sett::max_field_height - p.get_y(), p.get_x()};
    },
    [](Point &p) {
      return Point{field_sett::max_field_width - p.get_x(), field_sett::max_field_height - p.get_y()};
    },
    [](Point &p) {
      return Point{p.get_y(), field_sett::max_field_width - p.get_x()};
    }
};

RobotPoint qr_detect(cv::Mat qr, std::array<BoxMap, 3> &boxes_pos, std::pair<Point, Point> &pz) {
    cv::QRCodeDetector qd;
    std::string s = qd.detectAndDecode(qr);
    Point pz_p1 = letter2coordinat(s[1], s[3]);
    Point pz_p2 = letter2coordinat(s[5], s[7]);
    int rot = 0;
    if (fabs(pz_p1.get_y() - pz_p2.get_y()) > fabs(pz_p1.get_x() - pz_p2.get_x())) {
        if (pz_p1.get_y() < (field_sett::max_field_height / 2.)) {
            rot = 3;
        } else {
            rot = 1;
        }
    } else if (pz_p1.get_x() > (field_sett::max_field_height / 2.)) {
        rot = 2;
    }
    pz_p1 = rot2point[rot](pz_p1);
    pz_p2 = rot2point[rot](pz_p2);
    double d1 = pz_p1.get_y() - pz_p2.get_y();
    double d2 = pz_p1.get_x() - pz_p2.get_x();
    double ang = atan2(d1, d2);
    if (pz_p1.get_x() > pz_p2.get_x()) {
        ang *= -1;
    }
    double x_offset = field_sett::parking_zone_door_size * sin(M_PI - ang);
    double y_offset = field_sett::parking_zone_door_size * cos(M_PI - ang);
    pz = std::make_pair(pz_p1, Point{pz_p1.get_x() + x_offset, pz_p1.get_y() + y_offset});
    Point offset_box_rot[4] = {{0, 0}, {-field_sett::climate_box_width, 0}, {-field_sett::climate_box_width, -field_sett::climate_box_height}, {0, -field_sett::climate_box_width}};
    for (int i = 0; i < 3; i++) {
        Point p = letter2coordinat(std::min(s[10 + i * 9], s[14 + i * 9]), std::min(s[12 + i * 9], s[16 + i * 9]));
        boxes_pos[i] = BoxMap(rot2point[rot](p) + offset_box_rot[rot]);
    }
    Point robot_pos = pz.first + Point{field_sett::parking_zone_door_size * sin(ang) / 2., -field_sett::parking_zone_door_size * cos(ang) / 2.};
    return {robot_pos.get_x(), robot_pos.get_y(), ang - M_PI};
}
