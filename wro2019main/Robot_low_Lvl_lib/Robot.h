#pragma once

#include "OmniWheels.h"

class Robot
{
public:
	Robot();
	virtual void Start();
private:
	
};

class RobotGardener : public Robot
{
public:
	RobotGardener();
	void Start() override;
private:
	std::shared_ptr<OmniWheels> omni;
	
};