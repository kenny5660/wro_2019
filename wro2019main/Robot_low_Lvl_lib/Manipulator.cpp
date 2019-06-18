#include "Manipulator.h"
Manipulator::Manipulator(std::shared_ptr<Servo> servo_low, 
	std::shared_ptr<Servo> servo_up)
	: servo_low_(servo_low)
	, servo_up_(servo_up)
{
	servo_low_->Enable();
	//servo_up_->Enable();
}


std::shared_ptr<Servo> Manipulator::GetServoLow()
{
	return servo_low_;
}


std::shared_ptr<Servo> Manipulator::GetServoUp()
{
	return servo_up_;
}


Manipulator::~Manipulator()
{
	servo_low_->Disable();
	servo_up_->Disable();
}


void Manipulator::Home(bool wait, int time)
{
	servo_low_->SetDegrees(61, wait,time);
}


void Manipulator::Out(bool wait, int time)
{
	servo_low_->SetDegrees(165, wait, time);
}


void Manipulator::CatchRight(bool wait,int time)
{

	servo_up_->SetDegrees(188, wait,time);
}


void Manipulator::CatchLeft(bool wait, int time)
{
	servo_up_->SetDegrees(241, wait, time);
}


void Manipulator::Middle(bool wait /* = false */, int time /* = 0 */)
{
	servo_low_->SetDegrees(98, wait, time);
}
