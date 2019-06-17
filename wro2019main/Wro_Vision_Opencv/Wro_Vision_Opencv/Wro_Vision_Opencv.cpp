// Wro_Vision_Opencv.cpp : Этот файл содержит функцию "main". Здесь начинается и
// заканчивается выполнение программы.
//

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <VisionAlgs.h>
#include <filesystem>
#include <string>
#include <iostream>
#include <opencv2/imgproc.hpp>
const std::string path_small_cube("C:\\Users\\misha\\YandexDisk\\OneDrive\\Robots\\NImyrio\\WRO2019\\smallBox\\");



cv::Mat imgHSVTest;
int hMinTest = 0, hMaxTest = 255, sMinTest = 0, sMaxTest = 255, vMinTest = 0,
    vMaxTest = 255;
cv::Mat imgCannyTest;
int CannyTestSize;
cv::Mat imgHoughCirclesTest;
int min_distTest = 14, param1Test = 210, param2Test = 11, min_radiusTest = 0,
    max_radiusTest = 21;

void OnChangeTestTrackBar(int i, void* v) {
  cv::Mat out;
  inRange(imgHSVTest, cv::Scalar(hMinTest, sMinTest, vMinTest),
          cv::Scalar(hMaxTest, sMaxTest, vMaxTest), out);
  cv::imshow("TestPhoto", out);
}

void TestPhoto(cv::Mat img) {
  // Mat img = imread(name);
  cv::namedWindow("TestPhoto", cv::WINDOW_NORMAL);
  cv::resizeWindow("TestPhoto", 1024, 768);
  cv::cvtColor(img, imgHSVTest, cv::COLOR_BGR2HSV);
  cv::createTrackbar("HMin", "TestPhoto", &hMinTest, 255,
                       OnChangeTestTrackBar);
  cv::createTrackbar("hMax", "TestPhoto", &hMaxTest, 255, OnChangeTestTrackBar);
  cv::createTrackbar("sMin", "TestPhoto", &sMinTest, 255, OnChangeTestTrackBar);
  cv::createTrackbar("sMax", "TestPhoto", &sMaxTest, 255, OnChangeTestTrackBar);
  cv::createTrackbar("vMin", "TestPhoto", &vMinTest, 255, OnChangeTestTrackBar);
  cv::createTrackbar("vMax", "TestPhoto", &vMaxTest, 255, OnChangeTestTrackBar);
  /*namedWindow("Photo", WINDOW_NORMAL);
  resizeWindow("Photo", 600, 600);
  imshow("Photo", img);*/
  OnChangeTestTrackBar(0,nullptr);
}

int main() {
  cv::Mat img;
  for (const auto& entry :
       std::filesystem::directory_iterator(path_small_cube)) {
    std::cout << entry.path() << std::endl;
    img = cv::imread(entry.path().string());
    
    color_t color = VisionGetSmallBox(img);
    std::cout << color << color;
    while (true) {

      TestPhoto(img);

	 int k = cv::waitKey();
      if (k == 27) 
		{
           break;
      }
    }
  }
    

  
}
