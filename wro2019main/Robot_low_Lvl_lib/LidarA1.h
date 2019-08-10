#pragma once
#include "Pwm.h"
#include "Uart.h"
#include "RpLidar-sdk/include/rplidar.h"
class LidarA1 : public Lidar
{
public:
	enum class LidarMod
	{
		k2k  = 0,
		k4k  = 1,
		k8k  = 2,
		k4ks = 3
	};
	LidarA1(std::shared_ptr<Uart> uart, std::shared_ptr<Pwm> pwm_pin, LidarMod lidar_mode = LidarMod::k8k);
	void StartScan(double speed) override;
	void SetSpeed(double speed)override;
	float GetSpeedHz()override;
	bool GetScan(std::vector<Lidar::Point>& points)override;
	void StopScan()override;
private:
	
	std::shared_ptr<Uart> uart_;
	std::shared_ptr<Pwm> pwm_pin_;
	std::shared_ptr<rp::standalone::rplidar::RPlidarDriver> lidar_;
	LidarMod lidar_mode_ = LidarMod::k8k;
	const size_t kLidarPoints_[4] = { 2048, 4096, 8192, 4096 }; 
	const int kFreqPwm_ = 25000;
};