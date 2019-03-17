//
// Created by Danila on 17.03.2019.
//

#include <c++/thread>
#include <opencv2/imgproc.hpp>
#include "../test.h"

void open_test_jar() {
    std::system(("java -jar " + lidar_emulator_path + lidar_emulator_name).c_str());
}

TEST(StreamTest, getCorners) {
    std::thread thr(open_test_jar);
    std::vector<PolarPoint> p;
    const std::string name = "Get Corners";
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE );
    DebugFieldMat mat;
    cv::rectangle(mat, cv::Point(0, 0), cv::Point(debug_width_img, debug_height_img), cv::Scalar(255, 255,255 ), CV_FILLED);
    while (thr.joinable()) {
        if (!read(lidar_stream_name, p, "")) {
            std::vector<std::vector<Point>> corners = get_corners(p);
            cv::rectangle(mat, cv::Point(0, 0), cv::Point(debug_width_img, debug_height_img), cv::Scalar(0, 0,0 ), CV_FILLED);
            mat.zoom = 0;
            add_lines_img(mat, corners);
            imshow(name, mat);
            cv::waitKey(33);
        }
    }
    thr.join();
    cv::destroyWindow(name);
}
