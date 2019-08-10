#pragma once
#include <utility>
#include <memory>
#include <atomic>
class OpticalFlow
{
public:
	virtual std::pair<double, double> GetPos() = 0;
	virtual std::pair<double, double> GetRowPos() = 0;
	virtual void Reset() = 0;
	virtual ~OpticalFlow() {}
};

#ifdef HARDWERE_MODE
#include "HidMice.h"
#include "ADNS3080.h"
#else
//#include "RobotModel.h"
#endif