#include "Robot.h"
#include <CppUTest/CommandLineTestRunner.h>
#include <stdio.h>
#include <iostream>
#include "debug.h"
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
TEST(HardwareTestGroup, Camera_test_get_frames)
{
	Camera cam(0);
	std::shared_ptr<cv::Mat> frame;
	frame = cam.GetFrame();
	cv::imwrite("test_frame.jpg", *frame);
	std::this_thread::sleep_for(std::chrono::seconds(2));
	frame = cam.GetFrame();
	cv::imwrite("test_frame2.jpg", *frame);
}
TEST(HardwareTestGroup, Lidar_dump_to_file)
{	
	std::vector<PolarPoint> pps;
	robot->GetLidarPolarPoints(pps);
	save_ld_data(pps, "real_robot");
	return;
}

TEST(HardwareTestGroup, Lidar_test)
{	
	std::vector<PolarPoint> pps;
	robot->GetLidarPolarPoints(pps);
	pps.clear();
	robot->GetLidarPolarPoints(pps);
	return;
}
TEST(HardwareTestGroup, Servo_getDeg_test)
{	
	robot->GetMan()->GetServoLow()->Disable();
	int deg  = robot->GetMan()->GetServoLow()->GetDegrees();
	int deg2  = robot->GetMan()->GetServoLow()->GetDegrees();
	int deg3  = robot->GetMan()->GetServoLow()->GetDegrees();
	std::cout  << "deg = " << deg << " deg2 = " << deg2 << " deg3 = " << deg3 << std::endl;
}
TEST(HardwareTestGroup, Servo_setDeg_test)
{	
	const int d1 = 140, d2 = 220 , d3 = 260;
	robot->GetMan()->GetServoLow()->SetDegrees(d1,true);
	int deg  = robot->GetMan()->GetServoLow()->GetDegrees();
	DOUBLES_EQUAL(deg, d1, 3);
	robot->GetMan()->GetServoLow()->SetDegrees(d2, true);
	int deg2  = robot->GetMan()->GetServoLow()->GetDegrees();
	DOUBLES_EQUAL(deg2, d2,3);
	robot->GetMan()->GetServoLow()->SetDegrees(d3, true);
	int deg3  = robot->GetMan()->GetServoLow()->GetDegrees();
	DOUBLES_EQUAL(deg3, d3,3);
	std::cout  << "deg = " << deg << " deg2 = " << deg2 << " deg3 = " << deg3 << std::endl;
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
