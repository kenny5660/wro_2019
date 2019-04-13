//
// Created by Danila on 11.03.2019.
//

#include "test.h"
#include "../highLogic/debug.h"

TEST(DebugShowIMG, ShowInWindow) {
    cv::Mat img = cv::imread(img_path + "test_img.jpg", cv::IMREAD_COLOR);
    show_debug_img("DebugShowIMG_ShowInWindow", img);
}

TEST(DebugShowIMG, ClearLogsAndSave) {
    clear_logs();
    cv::Mat img = cv::imread(img_path + "test_img.jpg", cv::IMREAD_COLOR);
    save_debug_img("DebugShowIMG_Save", img);
}
