// Wro_Vision_Opencv.cpp : Этот файл содержит функцию "main". Здесь начинается и
// заканчивается выполнение программы.
//

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
const std::string path_small_cube("C:\\Users\\misha\\YandexDisk\\OneDrive\\Robots\\NImyrio\\WRO2019\\smallBox\\");
int main() {
  cv::Mat img;
  img = cv::imread(path_small_cube + "OLL2.jpg");
  std::cout << "Hello World!\n";
}
