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


void Manipulator::Home(bool wait)
{
	servo_low_->SetDegrees(60, wait);
}


void Manipulator::Out(bool wait)
{
	servo_low_->SetDegrees(160, wait);
}


void Manipulator::CatchRight(bool wait)
{

	servo_up_->SetDegrees(194, wait);
}


void Manipulator::CatchLeft(bool wait)
{
	servo_up_->SetDegrees(238, wait);
}
