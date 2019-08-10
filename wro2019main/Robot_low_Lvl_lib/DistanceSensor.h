#pragma once

class DistanceSensor
{
public:
	//@brief Getting distance in mm;
virtual	int GetDistance() = 0;
	virtual int GetRealDistance() = 0;
	virtual ~DistanceSensor(){};
};


#ifdef HARDWERE_MODE
#include "Sharp2_15.h"
#else
#include "RobotModel.h"
#endif
