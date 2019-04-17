#include <iostream>
#include <opencv2/highgui.hpp>
#include "MyRio_lib/DIO.h"
#include <chrono>
#include <thread>
#include "Robot.h"



int start_robot()
{
	RobotGardener robot;
	robot.GetOmni()->GetMotor(OmniWheels::MotorDir::FRONT)->MoveContinue(200);
	return 0;
}