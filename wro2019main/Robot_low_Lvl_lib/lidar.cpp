#include "lidar.h"
#include <cmath>
LidarA1::LidarA1(std::shared_ptr<Uart> uart, 
	std::shared_ptr<Pwm> pwm_pin,
	LidarMod lidar_mode)
	: uart_(uart),
	pwm_pin_(pwm_pin),
	lidar_mode_(lidar_mode)
{
}


void LidarA1::StartScan(double speed)
{
	//lidar = std::shared_ptr<rp::standalone::rplidar::RPlidarDriver>(rp::standalone::rplidar::RPlidarDriver::CreateDriver(uart_B,dtr_pin));
	
lidar_ = std::shared_ptr<rp::standalone::rplidar::RPlidarDriver>(rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT));
	lidar_->connect(uart_->GetFilePath().c_str(), 115200);
	lidar_->startMotor();
	pwm_pin_->Enable(kFreqPwm_, speed);
	std::vector<rp::standalone::rplidar::RplidarScanMode> scanModes;
	rp::standalone::rplidar::RplidarScanMode scanMode;
	int res  = lidar_->getAllSupportedScanModes(scanModes);
	if (IS_FAIL(res))
	{
		throw std::runtime_error("Lidar connection error!");
	}
	lidar_->startScanExpress(false, scanModes[(int)lidar_mode_].id, 0, &scanMode);
}


void LidarA1::SetSpeed(double speed)
{
	pwm_pin_->SetDuty(speed);
}


float LidarA1::GetSpeedHz()
{
	std::vector<Lidar::Point> points;
	GetScan(points);
	return kLidarPoints_[(int)lidar_mode_] / points.size();
}


bool LidarA1::GetScan(std::vector<Lidar::Point>& points)
{
	int res;
	rplidar_response_measurement_node_hq_t nodes[kLidarPoints_[(int)lidar_mode_]];
	size_t size_nodes = kLidarPoints_[(int)lidar_mode_];
	do
	{
		res = lidar_->grabScanDataHq(nodes, size_nodes);
	} while (IS_FAIL(res));
	lidar_->ascendScanData(nodes, size_nodes);
	
	for (int i = 0; i  < size_nodes; ++i)
	{
		float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
		float distance_in_mm = nodes[i].dist_mm_q2 / (1 << 2);
		Lidar::Point pp{distance_in_mm, (double)angle_in_degrees / 180 * M_PI,nodes[i].quality};
		points.push_back(pp);
	}
}


void LidarA1::StopScan()
{
	lidar_->disconnect();
	pwm_pin_->Disable();
}
