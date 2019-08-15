#pragma once
#include "Servo.h"

class Manipulator
{
public:
	Manipulator(std::shared_ptr<Servo> servo_low, std::shared_ptr<Servo> servo_up);
	void Home(bool wait = false, int time = 0);
	void Out(bool wait = false, int time = 0);
	void Out2(bool wait = false, int time = 0);
	void Middle(bool wait = false, int time = 0);
	void CatchLeft(bool wait = false, int time =0);
	void CatchRight(bool wait = false, int time = 0);
	std::shared_ptr<Servo> GetServoLow();
	std::shared_ptr<Servo> GetServoUp();
	~Manipulator();
private:
	std::shared_ptr<Servo> servo_low_;
	std::shared_ptr<Servo> servo_up_;
};

