//
// Created by Danila on 06.05.2019.
//

#include "test.h"
#include "../highLogic/CV.h"

TEST(CV, detectQR) {
    cv::Mat qr = cv::imread(img_path + "QR_K_Q_J_N_I_I_K_F_N_C_P_E_E_O_C_Q.png", cv::IMREAD_COLOR);
    std::array<BoxMap, 3> boxes;
    std::pair<Point, Point> pz;
    RobotPoint p = qr_detect(qr, boxes, pz);
    Map map(pz.first, pz.second, boxes, p);
    show_debug_img("", map.get_img());
}