#pragma once
class Indicator
{
public:
	enum DisplayEnum
	{ ERRORR = 0, GREEN, RED, D_BLUE, ORANGE, WHITE, OFF, YELLOW
	};

	virtual void Display(DisplayEnum disp) = 0;
	virtual ~Indicator() {}

};
#ifdef HARDWERE_MODE
#include "RgbLed.h"
#else
//#include "RobotModel.h"
#endif
