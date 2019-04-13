#include <iostream>
#include <opencv2/highgui.hpp>
#include "MyRio_lib/DIO.h"
#include <chrono>
#include <thread>




int start_robot()
{
	//Digital out example
	MyRio_Dio A0;
	A0.dir = DIOA_70DIR;
	A0.out = DIOA_70OUT;
	A0.in = DIOA_70IN;
	A0.bit = 0;
	MyRio_Dio A1;
	A1.dir = DIOA_70DIR;
	A1.out = DIOA_70OUT;
	A1.in = DIOA_70IN;
	A1.bit = 1;
	
	Dio_WriteBit(&A0, 1);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	Dio_WriteBit(&A0, 0);
	
	int a1 = Dio_ReadBit(&A1); 
	std::cout << "A1 = " << a1 << std::endl;
	
	//chek work opencv
	cv::Mat img_red(cv::Scalar(0, 0, 255));
	cv::imwrite("img.red.png", img_red);
	
	char sz[] = "img.red.png created!"; 	
	
	std::cout << sz << std::endl; 	//<================= Put a breakpoint here
	return 0;
}