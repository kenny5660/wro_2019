#pragma once

#include "OmniWheels.h"

class Robot
{
public:
	Robot();
	void Start();
private:
	
};

class RobotGardener : public Robot
{
public:
	RobotGardener();
private:
	std::shared_ptr<OmniWheels> omni;
	
};