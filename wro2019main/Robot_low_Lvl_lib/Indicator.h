#pragma once
#include "MyRio_lib/AIO.h"
#include <utility>
#include <memory>
#include <vector>
class Indicator
{
public:
	enum DisplayEnum
	{
		ERROR  = 0,
		GREEN,
		RED,
		D_BLUE,
		ORANGE,
		WHITE,
		OFF,
		YELLOW
	};

	virtual void Display(DisplayEnum disp);
	virtual ~Indicator() {}

};
#ifdef HARDWERE_MODE
#include "RgbLed.h"
#else
#include "RobotModel.h"
#endif
