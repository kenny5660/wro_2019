#include <iostream>
#include <opencv2/highgui.hpp>
#include "MyRio_lib/DIO.h"
#include <chrono>
#include <thread>
#include "Robot.h"



int start_robot()
{
	RobotGardener robot;
	robot.GetOmni()->Move(std::make_pair(15000, 0), 0);
	robot.Delay(700);
	robot.GetOmni()->Stop();
	robot.Delay(700);
	robot.GetOmni()->Move(std::make_pair(-15000, 15000), 0);
	robot.Delay(700);
	robot.GetOmni()->Stop();
	robot.Delay(700);
	robot.GetOmni()->Move(std::make_pair(15000, -15000), 0);
	robot.Delay(700);
	robot.GetOmni()->Stop();
	robot.Delay(700);
	robot.GetOmni()->Move(std::make_pair(0, 15000), 0);
	robot.Delay(700);
	robot.GetOmni()->Stop();
	robot.GetOmni()->Move(std::make_pair(20000, 0), 200);
	robot.Delay(1500);
	robot.GetOmni()->Stop();
	robot.Delay(700);
	return 0;
}