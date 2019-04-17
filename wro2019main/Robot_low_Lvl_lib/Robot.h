#pragma once

#include "OmniWheels.h"

class Robot
{
public:
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