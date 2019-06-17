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
const std::string path_small_cube("C:\\Users\\misha\\YandexDisk\\OneDrive\\Robots\\NImyrio\\WRO2019\\smallBox\\");
int main() {
  cv::Mat img;
  for (const auto& entry :
       std::filesystem::directory_iterator(path_small_cube)) {
    std::cout << entry.path() << std::endl;
    img = cv::imread(entry.path().string());
    color_t color = VisionGetSmallBox(img);
    std::cout << color << color;
    while (true) {

      cv::imshow();

	 int k = cv::waitKey(33);
      if (k == 27) 
		{
           break;
      }
    }
  }
    

  
}
