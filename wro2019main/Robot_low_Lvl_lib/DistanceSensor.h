#pragma once
#include "MyRio_lib/AIO.h"
#include <memory>
class DistanceSensor
{
public:
	//@brief Getting distance in mm;
virtual	int GetDistance() = 0;
	virtual ~DistanceSensor(){};
};

class Sharp2_15 : public DistanceSensor
{
public:
	Sharp2_15(std::shared_ptr<MyRio_Aio> analog_in);
	int GetDistance() override;
private:
	const double kPowerCoeffC = 4.03576E+4;
	const double kPowerCoeffP = -1.26093;
	std::shared_ptr<MyRio_Aio> analog_in_;
};

