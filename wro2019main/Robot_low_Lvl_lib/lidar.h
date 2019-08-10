#pragma once
#include <memory>
#include <vector>

class Lidar
{
public:
	struct Point
	{
		double r;
		double ph;
		int quality;
	};

	//@brief Start scan with speed. Must be call befor getScan
	//@param speed value in % (100% = 1, 50% = 0.5)
	virtual void StartScan(double speed) = 0;
	virtual void SetSpeed(double speed) = 0;
	virtual float GetSpeedHz() = 0;
	virtual bool GetScan(std::vector<Lidar::Point>& points) = 0;
	virtual void StopScan() = 0;
};
#ifdef HARDWERE_MODE
#include "LidarA1.h"
#else
#include "RobotModel.h"
#endif