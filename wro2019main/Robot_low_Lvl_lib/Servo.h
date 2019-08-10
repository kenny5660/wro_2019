#pragma once

#include <memory>
class Servo
{
public:
	virtual void Enable() = 0;
	virtual void Disable() = 0;
	virtual void SetDegrees(double deg,bool wait = false, uint16_t time = 0) = 0;
	virtual double GetDegrees() = 0;
	virtual int GetLead()  = 0;
	virtual bool IsLead() = 0;

};
#ifdef HARDWERE_MODE
#include "ServoOcs.h"
#else
//#include "RobotModel.h"
#endif