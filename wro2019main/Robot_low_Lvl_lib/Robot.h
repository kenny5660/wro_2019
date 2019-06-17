#pragma once

#include "OmniWheels.h"
#include "Indicator.h"
#include "DistanceSensor.h"
#include  "Manipulator.h"
#include "logic_structures.h"
#include "lidar.h"
#include "GPIO.h"
#include "Button.h"
#include "Camera.h"
#include  "OpticalFlow.h"
#include "map.h"
class Robot
{
public:
	virtual void Init() = 0;
	//@brief sleep(wait) 'msec' milliseconds
	//@param msec milliseconds to wait
		enum class CatchCubeSideEnum
	{
		LEFT,
		RIGHT,
		NONE
	}
	;
	enum DistSensorEnum
	{
		DIST_LEFT    = 0,
		DIST_TOP     = 1,
		DIST_C_RIGHT = 2,
		DIST_C_LEFT  = 3
	};
	
	virtual std::shared_ptr<DistanceSensor> GetDistSensor(DistSensorEnum dist_sensor) = 0;
	//@brief Ctach cube 
	//@param side is side of manipulator which will be filled after catch (empty side of manipulator before catch)
	virtual color_t CatchCube(CatchCubeSideEnum side, bool IsTakePhoto = true) = 0;
	virtual void Delay(int msec);
	virtual void Go2(std::vector<Point>) = 0;
	virtual CatchCubeSideEnum AlliginByDist(int dist, int offset_alg) = 0;
	virtual color_t GetColorFromAng(double ang) = 0;
	//@brief in radians
	virtual void Turn(double angle) = 0;
	virtual void GetQRCode(cv::Mat& frame) = 0;
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

	void WaitStartButton();
	color_t CatchCube(CatchCubeSideEnum side, bool IsTakePhoto = true) override;
	CatchCubeSideEnum AlliginByDist(int dist, int offset_alg) override;
	void GetQRCode(cv::Mat &frame) override;
	std::shared_ptr<OpticalFlow> GetOptFlow();
	std::shared_ptr<cv::Mat> GetQrCodeFrame();
	void Turn(double angle) override;
	void Go2(std::vector<Point>) override;
	void GetLidarPolarPoints(std::vector<PolarPoint>& polar_points) override;
	std::shared_ptr<DistanceSensor> GetDistSensor(DistSensorEnum dist_sensor) override;
	color_t GetColorFromAng(double ang) override;
	~RobotGardener();


private:
	void MouseTurn(double angle, int speed);
	void MoveByOptFlow(std::pair<int, int> toPos, double speed);
	void CatchLeft_();
	void CatchRight_();
	void AlliginHorizontal_(CatchCubeSideEnum side, CatchCubeSideEnum side_relative_cube);
	std::shared_ptr<CameraRotate> cam_rot_;
	std::shared_ptr<Manipulator> man_;
	std::shared_ptr<OmniWheels> omni_;
	std::shared_ptr<Indicator> indicator_;
	std::shared_ptr<DistanceSensor>dist_sensors_[4];
	std::shared_ptr<Lidar> lidar_;
	std::shared_ptr<Button> start_but_;
	std::shared_ptr<OpticalFlow> opt_flow_;
};