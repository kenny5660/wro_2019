//
// Created by Danila on 15.03.2019.
//

#include <iostream>
#include <fstream>
#include "test.h"

bool read(std::string s, std::vector<PolarPoint> &points, std::string path) {
    std::ifstream file(path + s);
    if (!file.is_open()) {
        //std::cerr << "File not found!" << std::endl;
        return true;
    }
    points.clear();
    double buff1;
    double buff2;
    while ((file >> buff1) && (file >> buff2)) {
        points.emplace_back(buff1, buff2);
    }
    file.close();
    return false;
}

std::string lines2string(const std::vector<std::vector<Point>> &p) {
    int accuracy = 1000;
    std::string s = "{";
    for (auto i : p) {
        s += "{";
        for (auto j : i) {
            s += "(";
            if (std::isnan(j.get_x())) {
                s += "nan";
            } else {
                s += std::to_string(
                    int(j.get_x() * accuracy) / double(accuracy));
            }
            s += ", ";
            if (std::isnan(j.get_y())) {
                s += "nan";
            } else {
                s += std::to_string(
                    int(j.get_y() * accuracy) / double(accuracy));
            }
            s += "), ";
        }
        s.pop_back();
        s.pop_back();
        s += "}, ";
    }
    s.pop_back();
    s.pop_back();
    s += "}";
    return s;
}
