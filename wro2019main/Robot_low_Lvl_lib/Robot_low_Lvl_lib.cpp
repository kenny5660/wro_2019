#include <iostream>
#include <opencv2/highgui.hpp>
#include "MyRio_lib/DIO.h"
#include <chrono>
#include <thread>
#include "Robot.h"
#include "debug.h"
#include "alg.h"
#include <csignal>


int start_robot()
{
	
	
	clear_logs();
	cout_to_file_log_enable();
	std::unique_ptr<RobotGardenerHardwere>	 robot(new RobotGardenerHardwere());
	try
	{
		robot->Init();
//		do_alg_code(*robot, true); // camicadze
		//do_alg_code(*robot, false);
		alg(*robot);
		
	}
	catch (std::runtime_error e)
	{
		std::cout << "Runtime error " << e.what() << std::endl;
		robot->GetIndicator()->Display(Indicator::RED);
		robot->Delay(1000);
	}

//	robot.~RobotGardener();

	return 0;
}