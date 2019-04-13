#include "test.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


TEST(OpenCV, IncludeOpenCV) {
    cv::Mat frame;
    frame = cv::imread(img_path + "test_img.jpg", cv::IMREAD_COLOR);
}
