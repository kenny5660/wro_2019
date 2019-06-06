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

    const int max_field_width = 2400 - (2 * 50);
    const int max_field_height = max_field_width;
    const int max_field = std::max(max_field_width, max_field_height);
    const int min_field = std::min(max_field_width, max_field_height);

    const int number_field_unit = 20;

    const double max_field_line = 10;
    const double size_field_unit = double(max_field_width) / number_field_unit;

    const int parking_zone_width_max = 380;
    const int parking_zone_width_min = 360;
    const int parking_zone_door_size = 345;
    const int parking_zone_thickness = 17;
    const int parking_zone_free_radius = 960 / 2;
    const double parking_zone_angel_min = atan2(1, 4);

    const int climate_box_width = 230;
    const int climate_box_height = climate_box_width;
    const int climate_box_max = std::max(climate_box_height, climate_box_width);
    const int climate_box_number_unit_offset = 4;
    const double climate_box_offset =
        field_sett::size_field_unit * climate_box_number_unit_offset;

    const int flower_cube_width = 48;
    const int flower_cube_height = flower_cube_width;
    const int flower_cube_max = std::max(flower_cube_height, flower_cube_width);
}

namespace lidar_sett {
    const int truncation_error = 5;
    const double max_tr_error = truncation_error * M_SQRT2;
    const double corner_step = 1 / M_PI;
    const double ang_death_start = 1.0001;// TODO
    const double ang_death_end = 1.0001;// TODO
}

namespace robot_sett {
    const double catch_flower_offset = 115;
    const double catch_offset_driveway = 80;
    const double move_offset = 170;
}

#endif //LIDAR_MATH_SETINGS_H
