#pragma once

#include "OmniWheels.h"
#include "Indicator.h"
#include "DistanceSensor.h"
#include  "Manipulator.h"
#include "logic_structures.h"
#include "lidar.h"
#include "GPIO.h"
#include "Camera.h"
class Robot
{
public:
	virtual void Init() = 0;
	//@brief sleep(wait) 'msec' milliseconds
	//@param msec milliseconds to wait
	virtual void Delay(int msec);
	virtual void GetLidarPolarPoints(std::vector<PolarPoint>& polar_points) = 0;
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
	std::shared_ptr<Manipulator> GetMan();
	std::shared_ptr<Lidar> GetLidar();
	std::shared_ptr<CameraRotate> GetCamRot();
	void CatchCube();
	void AlliginByDist(int dist,int offset_alg);
	void AlliginRight();
	std::shared_ptr<cv::Mat> GetQrCodeFrame();
	void GetLidarPolarPoints(std::vector<PolarPoint>& polar_points) override;
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

	std::shared_ptr<CameraRotate> cam_rot_;
	std::shared_ptr<Manipulator> man_;
	std::shared_ptr<OmniWheels> omni_;
	std::shared_ptr<Indicator> indicator_;
	std::shared_ptr<DistanceSensor>dist_sensors_[4];
	std::shared_ptr<Lidar> lidar_;

};