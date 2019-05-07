#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "Servo.h"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
class Camera
{
public:
	Camera(int num);
	Camera(std::shared_ptr<cv::VideoCapture> vc);
	void SetResolution(std::pair<int,int> res);
	std::shared_ptr<cv::VideoCapture> GetVc();
	virtual std::shared_ptr<cv::Mat> GetFrame();
	virtual ~Camera();
protected:
	std::shared_ptr<cv::VideoCapture> vc_;

private:
	std::shared_ptr<cv::Mat> frame_cur_;
	std::thread th_update_frame_;
	std::atomic<bool> stop_signal_th_;
	std::mutex mutex_update_frame_;
	void UpdateFrameThread();
};


class CameraRotate : public Camera
{
public:
	CameraRotate(int num, std::shared_ptr<Servo> servo_rot);
	CameraRotate(std::shared_ptr<cv::VideoCapture> vc, std::shared_ptr<Servo> servo_rot);
	std::shared_ptr<Servo> GetServo();
	void RotateTo(double deg);
	virtual std::shared_ptr<cv::Mat> GetFrame(double deg);
	virtual ~CameraRotate();
private:
	std::shared_ptr<Servo> servo_rot_;
};

class CameraDebug
{
public:
	CameraDebug(std::shared_ptr<Camera> cam);
	bool CreateServer(int port);
	void SendFrame();
	std::shared_ptr<Camera> GetCamera();
private:
	std::shared_ptr<Camera> cam_;
};

class CameraRotateDebug : CameraDebug
{
public:
	CameraRotateDebug(CameraRotate cam_rot);
	
private:
	std::shared_ptr<CameraRotate> cam_rot_;
};