#include <iostream>
#include <opencv2/highgui.hpp>
#include "MyRio_lib/DIO.h"
#include <chrono>
#include <thread>
#include "Robot.h"



int start_robot()
{
	RobotGardener robot;
	robot.Start();
	return 0;
}