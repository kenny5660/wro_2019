#pragma once
#include "Servo.h"

class Manipulator
{
public:
	Manipulator(std::shared_ptr<Servo> servo_low, std::shared_ptr<Servo> servo_up);
	void Home(bool wait = false);
	void Out(bool wait = false);
	void CatchLeft(bool wait = false);
	void CatchRight(bool wait = false);
	std::shared_ptr<Servo> GetServoLow();
	std::shared_ptr<Servo> GetServoUp();
	~Manipulator();
private:
	std::shared_ptr<Servo> servo_low_;
	std::shared_ptr<Servo> servo_up_;
};

