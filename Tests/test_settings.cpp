//
// Created by Danila on 15.02.2019.
//

#include "test.h"

TEST(SettingsCheck, FieldSettings) {
    ASSERT_EQ(field_sett::truncation_field_error, 5);

    EXPECT_NEAR(field_sett::max_field_width, 2400, field_sett::truncation_field_error);
    ASSERT_EQ(field_sett::max_field_height, field_sett::max_field_width);

    EXPECT_NEAR(field_sett::parking_zone_width_max, 380, field_sett::truncation_field_error);
    EXPECT_NEAR(field_sett::parking_zone_width_min, 360, field_sett::truncation_field_error);

    EXPECT_NEAR(field_sett::climate_box_width, 230, field_sett::truncation_field_error);
    ASSERT_EQ(field_sett::climate_box_width, field_sett::climate_box_height);

    EXPECT_NEAR(field_sett::flower_cube_width, 48, field_sett::truncation_field_error);
    ASSERT_EQ(field_sett::flower_cube_width, field_sett::flower_cube_height);
}

TEST(SettingsCheck, LidarSettings) {
    EXPECT_EQ(lidar_sett::truncation_error, 2);
    EXPECT_GT(lidar_sett::corner_step, 0);
}
