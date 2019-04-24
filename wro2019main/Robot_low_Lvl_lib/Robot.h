#pragma once

#include "OmniWheels.h"
#include "Indicator.h"
#include "DistanceSensor.h"

class Robot
{
public:
	virtual void Init() = 0;
	//@brief sleep(wait) 'msec' milliseconds
	//@param msec milliseconds to wait
	virtual void Delay(int msec);
	virtual  ~Robot();
private:
	
};

class RobotGardener : public Robot
{
public:
	RobotGardener();
	void Init() override;
	std::shared_ptr<OmniWheels> GetOmni();
	std::shared_ptr<Indicator> GetIndicator();
	enum DistSensorEnum
	{
		DIST_LEFT    = 0,
		DIST_RIGHT   = 1,
		DIST_C_RIGHT = 2,
		DIST_C_LEFT  = 3
	};
	std::shared_ptr<DistanceSensor> GetDistSensor(DistSensorEnum dist_sensor);
	~RobotGardener();
private:
	std::shared_ptr<OmniWheels> omni_;
	std::shared_ptr<Indicator> indicator_;
	std::shared_ptr<DistanceSensor>dist_sensors_[4];
};