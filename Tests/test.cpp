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
