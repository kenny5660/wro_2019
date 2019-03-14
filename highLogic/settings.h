//
// Created by Danila on 14.02.2019.
//
// Все растояния в мм.
// Все углы в радианах.
//

#ifndef LIDAR_MATH_SETINGS_H
#define LIDAR_MATH_SETINGS_H

#define TRUNCATION_FIELD_ERROR (5)

#define MAX_FIELD_WIDTH (2400)
#define MAX_FIELD_HEIGHT MAX_FIELD_WIDTH

#define PARKING_ZONE_WIDTH_MAX (380)
#define PARKING_ZONE_WIDTH_MIN (360)

#define CLIMATE_BOX_WIDTH (230)
#define CLIMATE_BOX_HEIGHT CLIMATE_BOX_WIDTH

#define FLOWER_CUBE_WIDTH (48)
#define FLOWER_CUBE_HEIGHT FLOWER_CUBE_WIDTH

#define TRUNCATION_LIDAR_ERROR (5)

#define LIDAR_CORNER_STEP (1 / M_PI)

namespace field_sett {
    const double max_field_line = 10;
    const int size_field_unit = 120;
}

#endif //LIDAR_MATH_SETINGS_H
