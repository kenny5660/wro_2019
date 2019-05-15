#include "Camera.h"


Camera::Camera(int num)
	: vc_(std::make_shared<cv::VideoCapture>(num))
	, frame_cur_(std::make_shared<cv::Mat>())
	, stop_signal_th_(false),
	th_update_frame_(&Camera::UpdateFrameThread, this)
	
{
	if (!vc_->isOpened())
	{
		throw std::runtime_error("Error, camera isn't opened!");
	}

}


Camera::Camera(std::shared_ptr<cv::VideoCapture> vc)
	: vc_(vc)
	, frame_cur_(std::make_shared<cv::Mat>())
	, stop_signal_th_(false),
	th_update_frame_(&Camera::UpdateFrameThread, this)
{
	if (!vc_->isOpened())
	{
		throw std::runtime_error("Error, camera isn't opened!");
	}
}


std::shared_ptr<cv::VideoCapture> Camera::GetVc()
{
	return vc_;
}

void Camera::UpdateFrameThread()
{
	while (!stop_signal_th_)
	{
		mutex_update_frame_.lock();
		(*vc_) >> (*frame_cur_);
		mutex_update_frame_.unlock();
		cv::waitKey(50);
	}
}

std::shared_ptr<cv::Mat> Camera::GetFrame()
{
	std::shared_ptr<cv::Mat> mat(std::make_shared<cv::Mat>());
	mutex_update_frame_.lock();
	frame_cur_->copyTo(*mat);
	mutex_update_frame_.unlock();
	return mat;
}


Camera::~Camera()
{
	stop_signal_th_ = true;
	th_update_frame_.join();
}

CameraRotate::CameraRotate(int num, 
	std::shared_ptr<Servo> servo_rot)
	: Camera(num)
	, servo_rot_(servo_rot)
	
{
}


CameraRotate::CameraRotate(std::shared_ptr<cv::VideoCapture> vc, 
	std::shared_ptr<Servo> servo_rot)
	: Camera(vc)
	, servo_rot_(servo_rot)
{
}


std::shared_ptr<Servo> CameraRotate::GetServo()
{
	return servo_rot_;
}


void CameraRotate::RotateTo(double deg)
{
	servo_rot_->SetDegrees(deg,true);
}


std::shared_ptr<cv::Mat> CameraRotate::GetFrame(double deg)
{
	RotateTo(deg);
	return Camera::GetFrame();
}





void Camera::SetResolution(std::pair<int,int> res)
{
	mutex_update_frame_.lock();
	vc_->set(cv::CAP_PROP_FRAME_WIDTH, res.first);
	vc_->set(cv::CAP_PROP_FRAME_HEIGHT, res.second);
	mutex_update_frame_.unlock();
}


CameraRotate::~CameraRotate()
{
	servo_rot_->Disable();
}
