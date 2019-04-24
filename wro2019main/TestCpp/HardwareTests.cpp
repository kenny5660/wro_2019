#include <CppUTest/CommandLineTestRunner.h>
#include <stdio.h>
#include "Robot.h"
#include <iostream>

/*
	This is a very basic sample demonstrating the CppUTest framework.
	Read more about CppUTest syntax here: https://cpputest.github.io/manual.html
*/

TEST_GROUP(DemoTestGroup)
{
};

TEST(DemoTestGroup, FailingTest)
{
    LONGS_EQUAL(1, 1);
    LONGS_EQUAL(1, 2);	//<= This test should fail here
}

TEST(DemoTestGroup, SuccessfulTest1)
{
	//This test should succeed
	UtestShell::getCurrent()->print("Hello from Test #1", __FILE__, __LINE__);
    LONGS_EQUAL(1, 1);
}

TEST(DemoTestGroup, SuccessfulTest2)
{
	//This test should succeed;
	printf("Hello from Test #2");
}

TEST_GROUP(HardwareTestGroup)
{
	std::shared_ptr<RobotGardener> robot;
	
	void setup()
	{
		IGNORE_ALL_LEAKS_IN_TEST();
		robot = std::shared_ptr<RobotGardener> (new RobotGardener());
		robot->Init();
	}
};


TEST(HardwareTestGroup, Init_robot_test)
{	
}
TEST(HardwareTestGroup, Dist_sensors_test)
{
	while (1)
	{
		std::cout  << "Dist left = " << robot->GetDistSensor(RobotGardener::DIST_LEFT)->GetDistance() << std::endl;
		std::cout  << "Dist right = " << robot->GetDistSensor(RobotGardener::DIST_RIGHT)->GetDistance() << std::endl;
		std::cout  << "Dist center left = " << robot->GetDistSensor(RobotGardener::DIST_C_LEFT)->GetDistance() << std::endl;
		std::cout  << "Dist center right = " << robot->GetDistSensor(RobotGardener::DIST_C_RIGHT)->GetDistance() << std::endl;
		robot->Delay(100);
	}
}
TEST(HardwareTestGroup, Indicator_led_test)
{
//	RobotGardener robot;
//	robot->Init();
	robot->GetIndicator()->Display(Indicator::RED);
	robot->Delay(1000);
	robot->GetIndicator()->Display(Indicator::GREEN);
	robot->Delay(1000);
	robot->GetIndicator()->Display(Indicator::D_BLUE);
	robot->Delay(1000);
	robot->GetIndicator()->Display(Indicator::ORANGE);
	robot->Delay(1000);
	robot->GetIndicator()->Display(Indicator::WHITE);
	robot->Delay(1000);
}

TEST(HardwareTestGroup, Motors_for_omni_test)
{
//	RobotGardener robot;
//	robot->Init();
	std::shared_ptr<Motor> motor_front(robot->GetOmni()->GetMotor(OmniWheels::MotorDir::FRONT));
	std::shared_ptr<Motor> motor_left(robot->GetOmni()->GetMotor(OmniWheels::MotorDir::LEFT));
	std::shared_ptr<Motor> motor_back(robot->GetOmni()->GetMotor(OmniWheels::MotorDir::BACK));
	std::shared_ptr<Motor> motor_right(robot->GetOmni()->GetMotor(OmniWheels::MotorDir::RIGHT));
	
	const int kDegs = 360;
	motor_front->MoveIncDeg(200, kDegs, true);
	DOUBLES_EQUAL((double)kDegs, (double)motor_front->GetCurEncDeg(), 5);
	motor_left->MoveIncDeg(200, kDegs, true);
	DOUBLES_EQUAL((double)kDegs, (double)motor_left->GetCurEncDeg(),5);
	motor_back->MoveIncDeg(200, kDegs, true);
	DOUBLES_EQUAL((double)kDegs, (double)motor_back->GetCurEncDeg(), 5);
	motor_right->MoveIncDeg(200, kDegs, true);
	DOUBLES_EQUAL((double)kDegs, (double)motor_right->GetCurEncDeg(), 5);
		
	robot->Delay(500);
}
