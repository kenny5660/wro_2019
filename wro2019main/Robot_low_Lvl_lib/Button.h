#pragma once


class Button
{
public:
	virtual void  WaitDown() = 0;
	virtual bool IsDown() = 0;
	virtual ~Button() {};
};
#ifdef HARDWERE_MODE
#include "ButtonOnMyrio.h"
#else
#include "RobotModel.h"
#endif