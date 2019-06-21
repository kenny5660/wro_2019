#pragma once
#include "Motor.h"

class OmniWheels
{
public:
	enum class MotorDir
	{
	    FRONT = 0, LEFT = 1,BACK = 2,RIGHT= 3    
	};
	// @brief Move wheels continualy
	// @param vSpeed vector(x,y) of speed
	// @param angular_speed  angular velocity
	virtual void MoveWithSpeed(std::pair<double, double> vSpeed, double angular_speed){throw std::runtime_error("Not implemented");}
	virtual void MoveToPosInc(std::pair<double, double> vSpeed, double speed_limit) = 0;
	virtual void MoveTrajectory(const std::vector<std::pair<int, int>> &tr, double speed) = 0;
	virtual void Turn(double angl, int speed) = 0;
	virtual std::pair<double,double> GetPosMm() = 0;
	virtual void Reset() = 0;
	// stop all wheels
	virtual void Stop(){throw std::runtime_error("Not implemented");}
	virtual std::shared_ptr<Motor> GetMotor(MotorDir motor_dir){throw std::runtime_error("Not implemented");}
	virtual ~OmniWheels() { };
};

class OmniWheels4Squre : public OmniWheels
{
public:
	OmniWheels4Squre(double r_wheel,
		double r_body,
		std::shared_ptr<Motor>motor_left,
		std::shared_ptr<Motor>motor_front, 
		std::shared_ptr<Motor>motor_right,
		std::shared_ptr<Motor>motor_back);
	// @brief Move wheels continualy
	// @param vSpeed vector(x,y) of speed, X axis is directed to right motor, Y axis is directed to front motor
	// @param angular_speed  angular velocity
	void MoveWithSpeed(std::pair<double, double> vSpeed, double angular_speed) override;
	void MoveToPosInc(std::pair<double, double> vSpeed, double speed_limit) override;
	void Turn(double angl, int speed) override;
	void Reset() override;
	void MoveTrajectory(const std::vector<std::pair<int, int>> &tr, double speed) override;
	void Stop() override;
	std::pair<double, double> GetPosMm() override;
	std::shared_ptr<Motor> GetMotor(MotorDir motor_dir) override;
private:
	double r_wheel_;
	double r_body_;
	std::shared_ptr<Motor> motors[4];
};