#include "OmniWheels.h"
#include <cmath>
OmniWheels4Squre::OmniWheels4Squre(
	int r_wheel,
	int r_body,
	std::shared_ptr<Motor>motor_front, 
	std::shared_ptr<Motor>motor_left, 
	std::shared_ptr<Motor>motor_back, 
	std::shared_ptr<Motor>motor_right)
	: r_wheel_(r_wheel)
	, r_body_(r_body)
	, motors{motor_left, motor_front, motor_right, motor_back}
{
	
}

//formuls is from picture http://www-ist.massey.ac.nz/conferences/ICARA2004/files/Papers/Paper74_ICARA2004_425_428.pdf
void OmniWheels4Squre::Move(std::pair<double, double> vSpeed, double angular_speed)
{
	angular_speed = angular_speed / 180 * M_PI;
	int w_front = ((r_body_*angular_speed - vSpeed.first) / r_wheel_) * 180 / M_PI;   //w1 on picture 
	int w_left = ((r_body_*angular_speed  -  vSpeed.second) / r_wheel_) * 180 / M_PI;  //w2 on picture 
	int w_back = ((r_body_*angular_speed  + vSpeed.first) / r_wheel_) * 180 / M_PI;  //w3 on picture 
	int w_right = ((r_body_*angular_speed + vSpeed.second) / r_wheel_) * 180 / M_PI;  //w4 on picture 
	
	motors[(int)MotorDir::FRONT]->MoveContinue(w_front);
	motors[(int)MotorDir::LEFT]->MoveContinue(w_left);
	motors[(int)MotorDir::BACK]->MoveContinue(w_back);
	motors[(int)MotorDir::RIGHT]->MoveContinue(w_right);
}


void OmniWheels4Squre::Stop()
{
	motors[(int)MotorDir::FRONT]->Stop();
	motors[(int)MotorDir::LEFT]->Stop();
	motors[(int)MotorDir::BACK]->Stop();
	motors[(int)MotorDir::RIGHT]->Stop();
}


std::shared_ptr<Motor> OmniWheels4Squre::GetMotor(MotorDir motor_dir)
{
	return motors[(int)motor_dir];
}
