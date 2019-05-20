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
	cout_to_file_log_enable();
		robot.Init();
	do_alg_code(robot, true);
	return 0;
}