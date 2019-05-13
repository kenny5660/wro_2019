#include "Robot.h"
#include <CppUTest/CommandLineTestRunner.h>
#include <stdio.h>
#include <iostream>
#include "debug.h"

#include <opencv2/core.hpp>
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
	void teardown()
	{
		
	}
};


TEST(HardwareTestGroup, Init_robot_test)
{	
}

void display(cv::Mat &im, cv::Mat &bbox, std::string str) {
	using namespace cv;
	int n = bbox.rows;
	for (int i = 0; i < n; i++) {
		line(
		    im,
			cv::Point2i(bbox.at<float>(i, 0), bbox.at<float>(i, 1)),
			Point2i(bbox.at<float>((i + 1) % n, 0), bbox.at<float>((i + 1) % n, 1)),
			Scalar(255, 0, 0),
			3);
	}
	cv::putText(im,
		str,
		cv::Point2i(bbox.at<float>(0, 0) - 50, bbox.at<float>(0, 1) - 15),
		FONT_ITALIC,
		0.6,
		Scalar(0, 0, 255));
	// imshow("Result", im);
}
TEST(HardwareTestGroup, Qrcode_get_test)
{	std::shared_ptr<cv::Mat> frame = robot->GetQrCodeFrame();
	cv::QRCodeDetector QrDetector;
	cv::Mat bbox, rectifiedImage;
	cv::imwrite("Qrcode_test.jpg", *frame);
	std::string str = QrDetector.detectAndDecode(*frame, bbox, rectifiedImage);
	CHECK(true);
	CHECK(str.length() > 0);
	display(*frame, bbox, str);
	cv::imwrite("Qrcode_test_with_text.jpg", *frame);
	
}

TEST(HardwareTestGroup, Camera_test_get_frames)
{
	std::shared_ptr<CameraRotate> cam_rot = robot->GetCamRot();
	std::shared_ptr<cv::Mat> frame;
	frame = cam_rot->Camera::GetFrame();
	cv::imwrite("test_frame.jpg", *frame);
	std::this_thread::sleep_for(std::chrono::seconds(2));
	frame = cam_rot->Camera::GetFrame();
	cv::imwrite("test_frame2.jpg", *frame);
}
TEST(HardwareTestGroup, Lidar_dump_to_file)
{	
	std::vector<PolarPoint> pps;
	robot->GetLidarPolarPoints(pps);
	save_ld_data(pps, "real_robot");
	DebugFieldMat mat;
	cv::rectangle(mat, cv::Point(0, 0), cv::Point(debug_width_img, debug_height_img), cv::Scalar(0, 0, 0), cv::FILLED);
	mat.zoom = 0;
	std::vector<Point> buff;
	for (auto i : pps) {
		buff.push_back(i.to_cartesian(-M_PI, true));
	}
	add_points_img(mat, buff,cv::Scalar(0,255,0));
	cv::imwrite("lidar_dump.jpg", mat);
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
	robot->GetMan()->GetServoUp()->Disable();
	robot->GetCamRot()->GetServo()->Disable();
	int degLow  = robot->GetMan()->GetServoLow()->GetDegrees();
	int degUp = robot->GetMan()->GetServoUp()->GetDegrees();
	int deg_up = robot->GetCamRot()->GetServo()->GetDegrees();//268
		
	int deg2Low  = robot->GetMan()->GetServoLow()->GetDegrees();
	int deg2Up  = robot->GetMan()->GetServoUp()->GetDegrees();
	int deg2_up = robot->GetCamRot()->GetServo()->GetDegrees();
	
	int deg3Low  = robot->GetMan()->GetServoLow()->GetDegrees();
	int deg3Up  = robot->GetMan()->GetServoUp()->GetDegrees();
	int deg3_up = robot->GetCamRot()->GetServo()->GetDegrees();
	
	std::cout  << "degLow = " << degLow << " deg2Low = " << deg2Low << " deg3Low = " << deg3Low << std::endl;
}
TEST(HardwareTestGroup, Mnipulator_test)
{	
	robot->GetMan()->Out();
	robot->GetMan()->CatchLeft();
	robot->GetMan()->Home();
	robot->GetMan()->CatchRight();
}
TEST(HardwareTestGroup, Servo_setDeg_test)
{	
	const int d1 = 60, d2 = 159 , d3 = 180;
	robot->GetMan()->GetServoLow()->SetDegrees(d1,true);
	int deg  = robot->GetMan()->GetServoLow()->GetDegrees();
	DOUBLES_EQUAL(deg, d1, 3);
	robot->GetMan()->GetServoLow()->SetDegrees(d2, true);
	int deg2  = robot->GetMan()->GetServoLow()->GetDegrees();
	DOUBLES_EQUAL(deg2, d2,3);
	robot->GetMan()->GetServoLow()->SetDegrees(d3, true);
	int deg3  = robot->GetMan()->GetServoLow()->GetDegrees();
	DOUBLES_EQUAL(deg3, d3,3);
	robot->GetMan()->GetServoLow()->SetDegrees(d1, true);

	std::cout  << "deg = " << deg << " deg2 = " << deg2 << " deg3 = " << deg3 << std::endl;
}
TEST(HardwareTestGroup, Omni_move_speed_test)
{
	robot->GetOmni()->MoveWithSpeed(std::make_pair(0, 230), 0);
    robot->Delay(1000);
	robot->GetOmni()->Stop();
	robot->GetOmni()->MoveWithSpeed(std::make_pair(230, 0), 0);
	robot->Delay(1000);
	robot->GetOmni()->Stop();
	robot->GetOmni()->MoveWithSpeed(std::make_pair(0, 0), 90);
	robot->Delay(1000);
	robot->GetOmni()->Stop();
}

TEST(HardwareTestGroup, Omni_move_pos_inc_test)
{
	const int speed = 200;
	robot->GetOmni()->MoveToPosInc(std::make_pair(0, 230), speed);
	robot->GetOmni()->MoveToPosInc(std::make_pair(230, 0), speed);
	robot->GetOmni()->MoveToPosInc(std::make_pair(-230, -230), speed);
}

TEST(HardwareTestGroup, Omni_move_trajectory_test)
{
	const int speed = 200;
	std::vector<std::pair<int, int>> traj = { 
		{0,115},
		{0, 115},
		{0, 115},
		{0, 115}
//		{0, -115},
//		{0, -115},
//		{0, -115},
//		{0, -115}
		};
	
	robot->GetOmni()->MoveTrajectory(traj, speed);
}

TEST(HardwareTestGroup, Aligin_by_Dist_test)
{
	robot->AlliginByDist(66,-4);
}
TEST(HardwareTestGroup, CatchCube_Left_test)
{
	robot->CatchCube(RobotGardener::CatchCubeSideEnum::LEFT);
	robot->Delay(500);
}
TEST(HardwareTestGroup, Dist_sensors_test)
{
	robot->GetMan()->GetServoLow()->Disable();
	robot->GetMan()->GetServoUp()->Disable();
	
	while (1)
	{
		std::cout  << "Dist left = " << robot->GetDistSensor(RobotGardener::DIST_LEFT)->GetDistance() << std::endl;
		std::cout  << "Dist top = " << robot->GetDistSensor(RobotGardener::DIST_TOP)->GetDistance() << std::endl;
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
