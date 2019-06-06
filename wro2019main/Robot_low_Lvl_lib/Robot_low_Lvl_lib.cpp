#include <iostream>
#include <opencv2/highgui.hpp>
#include "MyRio_lib/DIO.h"
#include <chrono>
#include <thread>
#include "Robot.h"
#include "debug.h"
#include "alg.h"
#include <csignal>

RobotGardener robot;
void stop_robot(int signal)
{
	std::cout << "Stop_signal " << signal;
	robot.~RobotGardener();
}

int start_robot()
{
	cout_to_file_log_enable();
		robot.Init();
	std::signal(SIGINT, stop_robot);
	do_alg_code(robot, true);
	robot.~RobotGardener();
	return 0;
}