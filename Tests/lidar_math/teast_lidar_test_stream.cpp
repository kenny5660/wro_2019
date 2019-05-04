//
// Created by Danila on 17.03.2019.
//

#include <c++/thread>
#include <opencv2/imgproc.hpp>
#include "../test.h"

void open_test_jar() {
    std::system(("java -jar " + lidar_emulator_path + lidar_emulator_name).c_str());
}

TEST(StreamTest, DrawPoint) {
    std::vector<PolarPoint> p;
    DebugFieldMat mat;
    const std::string name = "Draw Point";
    while (1) {
        if (!read(lidar_stream_name, p, "")) {
            cv::rectangle(mat, cv::Point(0, 0), cv::Point(debug_width_img, debug_height_img), cv::Scalar(0, 0,0 ), CV_FILLED);
            mat.zoom = 0;
            std::vector<Point> buff;
            for (auto i : p) {
                buff.push_back(i.to_cartesian(-M_PI, true));
            }
            add_points_img(mat, buff);
            imshow(name, mat);
            cv::waitKey(33);
        }
    }
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

TEST(StreamTest, detectTypeLine) {
    std::thread thr(open_test_jar);
    std::vector<PolarPoint> p;
    const std::string name = "Get Corners";
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE );
    DebugFieldMat mat;
    cv::rectangle(mat,
                  cv::Point(0, 0),
                  cv::Point(debug_width_img, debug_height_img),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
    while (thr.joinable()) {
        if (!read(lidar_stream_name, p, "")) {
            auto corners = line2line_type(get_corners(p));
            detect_boarder(corners);
            detected_parking_zone(corners);
            detected_box(corners);
            cv::rectangle(mat,
                          cv::Point(0, 0),
                          cv::Point(debug_width_img, debug_height_img),
                          cv::Scalar(0, 0, 0),
                          CV_FILLED);
            mat.zoom = 0;
            add_lines_img(mat, corners);
            imshow(name, mat);
            cv::waitKey(33);
        }
    }
    thr.join();
    cv::destroyWindow(name);
}

TEST(StreamTest, CreatMap) {
    std::thread thr(open_test_jar);
    std::vector<PolarPoint> p;
    const std::string name = "Creat Map";
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE );
    while (thr.joinable()) {
        if (!read(lidar_stream_name, p, "")) {
            //Map m(p, show_debug_img);
            Map m(p);
            imshow(name, m.get_img(400, 400));
            std::cout << "X: " << m.get_position().get_x()
                      << " Y: " << m.get_position().get_y()
                      << " Ang: " << m.get_position().get_angle() * 180 / M_PI << std::endl;
            cv::waitKey(33);
        }
    }
    thr.join();
    cv::destroyWindow(name);
}
