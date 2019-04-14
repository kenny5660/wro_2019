#include "OmniWheels.h"

OmniWheels4Squre::OmniWheels4Squre(
	int r_wheel,
	int r_body,
	std::shared_ptr<Motor>motor_front, 
	std::shared_ptr<Motor>motor_left, 
	std::shared_ptr<Motor>motor_back, 
	std::shared_ptr<Motor>motor_right)
	: r_wheel_(r_wheel)
	, r_body_(r_body)
	, motor_left_(motor_left)
	, motor_front_(motor_front)
	, motor_right_(motor_right)
	, motor_back_(motor_back)
{
	
}

//formuls is from picture http://www-ist.massey.ac.nz/conferences/ICARA2004/files/Papers/Paper74_ICARA2004_425_428.pdf
void OmniWheels4Squre::Move(std::pair<double, double> vSpeed, double angular_speed)
{
	int w_front = (r_body_*angular_speed - vSpeed.first) / r_wheel_;  //w1 on picture 
	int w_left = (r_body_*angular_speed  -  vSpeed.second) / r_wheel_; //w2 on picture 
	int w_back = (r_body_*angular_speed  + vSpeed.first) / r_wheel_; //w3 on picture 
	int w_right = (r_body_*angular_speed + vSpeed.second) / r_wheel_; //w4 on picture 
	
	motor_front_->MoveContinue(w_front);
	motor_left_->MoveContinue(w_left);
	motor_back_->MoveContinue(w_back);
	motor_right_->MoveContinue(w_right);
}
