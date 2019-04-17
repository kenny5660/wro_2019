#include <gtest/gtest.h>
#include <stdio.h>
#include "Robot.h"
/*
	This is a very basic sample demonstrating the GoogleTest framework.
	Read more about CppUTest syntax here: https://github.com/google/googletest
*/

TEST(HardwareTestGroup, Motors_for_omni_test)
{
	RobotGardener robot;
	std::shared_ptr<Motor> motor_front(robot.GetOmni()->GetMotor(OmniWheels::MotorDir::FRONT));
	std::shared_ptr<Motor> motor_left(robot.GetOmni()->GetMotor(OmniWheels::MotorDir::LEFT));
	std::shared_ptr<Motor> motor_back(robot.GetOmni()->GetMotor(OmniWheels::MotorDir::BACK));
	std::shared_ptr<Motor> motor_right(robot.GetOmni()->GetMotor(OmniWheels::MotorDir::RIGHT));
	
	const int kDegs = 360;
	motor_front->MoveIncDeg(200, kDegs, true);
	EXPECT_EQ(kDegs, motor_front->GetCurEncDeg());
	motor_left->MoveIncDeg(200, kDegs, true);
	EXPECT_EQ(kDegs, motor_left->GetCurEncDeg());
	motor_back->MoveIncDeg(200, kDegs, true);
	EXPECT_EQ(kDegs, motor_back->GetCurEncDeg());
	motor_right->MoveIncDeg(200, kDegs, true);
	EXPECT_EQ(kDegs, motor_right->GetCurEncDeg());
		
	robot.Delay(500);
}
