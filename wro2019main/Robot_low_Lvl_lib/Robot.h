#pragma once

#include "OmniWheels.h"

class Robot
{
public:
	//@brief sleep(wait) 'msec' milliseconds
	//@param msec milliseconds to wait
	void Delay(int msec);
private:
	
};

class RobotGardener : public Robot
{
public:
	RobotGardener();
	std::shared_ptr<OmniWheels> GetOmni();
	
private:
	std::shared_ptr<OmniWheels> omni;
};