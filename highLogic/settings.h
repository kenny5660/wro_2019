//
// Created by Danila on 14.02.2019.
//
// Все растояния в мм.
// Все углы в радианах.
//

#ifndef LIDAR_MATH_SETINGS_H
#define LIDAR_MATH_SETINGS_H

namespace field_sett {
    const int truncation_field_error = 5;

    const int max_field_width = 2400;
    const int max_field_height = max_field_width;

    const int number_field_unit = 20;

    const double max_field_line = 10;
    const double size_field_unit = double(max_field_width) / number_field_unit;

    const int parking_zone_width_max = 380;
    const int parking_zone_width_min = 360;

    const int climate_box_width = 230;
    const int climate_box_height = climate_box_width;

    const int flower_cube_width = 48;
    const int flower_cube_height = flower_cube_width;
}

namespace lidar_sett {
    const int truncation_error = 5;
    const double corner_step = 1 / M_PI;
}

#endif //LIDAR_MATH_SETINGS_H
