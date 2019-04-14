#pragma once
#include "Motor.h"
class OmniWheels
{
	// @brief Move wheels continualy
	// @param vSpeed vector(x,y) of speed
	// @param angular_speed  angular velocity
	virtual void Move(std::pair<double, double> vSpeed, double angular_speed);
	// stop all wheels
	void Stop();
};

class OmniWheels4Squre : public OmniWheels
{
public:
	OmniWheels4Squre(int r_wheel,
		int r_body,
		std::shared_ptr<Motor>motor_left,
		std::shared_ptr<Motor>motor_front, 
		std::shared_ptr<Motor>motor_right,
		std::shared_ptr<Motor>motor_back);
	// @brief Move wheels continualy
	// @param vSpeed vector(x,y) of speed, X axis is directed to right motor, Y axis is directed to front motor
	// @param angular_speed  angular velocity
	void Move(std::pair<double, double> vSpeed, double angular_speed) override;
private:
	int r_wheel_;
	int r_body_;

	std::shared_ptr<Motor> motor_front_;
	std::shared_ptr<Motor> motor_left_;
	std::shared_ptr<Motor> motor_back_;
	std::shared_ptr<Motor> motor_right_;

};