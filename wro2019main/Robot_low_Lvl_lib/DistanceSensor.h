#pragma once
#include "MyRio_lib/AIO.h"
#include <memory>
#include <MedianFilter/MedianFilter.h>
class DistanceSensor
{
public:
	//@brief Getting distance in mm;
virtual	int GetDistance() = 0;
	virtual int GetRealDistance() = 0;
	virtual ~DistanceSensor(){};
};

class Sharp2_15 : public DistanceSensor
{
public:
	Sharp2_15(std::shared_ptr<MyRio_Aio> analog_in, int median_filter_w_size = 5, int median_filter_seed = 5000);
	int GetDistance() override;
	void SkipWindow();
	int GetRealDistance() override;
private:
	const double kPowerCoeffC = 4.03576E+4;
	const double kPowerCoeffP = -1.26093;
	std::shared_ptr<MyRio_Aio> analog_in_;
	MedianFilter m_filter_;
};

