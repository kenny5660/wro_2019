#include <iostream>
#include <opencv2/highgui.hpp>
#include "MyRio_lib/DIO.h"
#include <chrono>
#include <thread>
#include "Robot.h"
#include "debug.h"
#include "alg.h"

int start_robot()
{
	RobotGardener robot;
	try
	{
	

		robot.Init();
	}
		catch (std::runtime_error e)
	{
		//TO DO output to log file
	}
	
	do_alg_code(robot);
	return 0;
}