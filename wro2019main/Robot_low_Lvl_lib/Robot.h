#pragma once

#include "OmniWheels.h"
#include "Indicator.h"
class Robot
{
public:
	virtual void Init();
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
	~RobotGardener();
private:
	std::shared_ptr<OmniWheels> omni_;
	std::shared_ptr<Indicator> indicator_;
};