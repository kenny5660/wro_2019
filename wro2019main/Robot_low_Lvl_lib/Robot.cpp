#include "Robot.h"
#include "Uart.h"
#include "Pwm.h"
#include  <exception>
extern NiFpga_Session myrio_session;
RobotGardener::RobotGardener()
{
	NiFpga_Status status_rio = MyRio_Open();
	if (MyRio_IsNotSuccess(status_rio)) {
		throw std::runtime_error("Error open MyRio!");
	}
	
}

std::shared_ptr<OmniWheels> RobotGardener::GetOmni()
{
	return omni_;
}

void Robot::Delay(int msec)
{
	struct timespec tw = { msec / 1000, (msec % 1000) * 1000000 };
	struct timespec tr;
	nanosleep(&tw, &tr);
}


void RobotGardener::Init()
{
	
	std::shared_ptr<Uart> uart_A(new MyRioUart(MyRioUart::UART_A, 115200));
	std::shared_ptr<Uart> uart_B(new MyRioUart(MyRioUart::UART_B, 115200));
	std::shared_ptr<Pwm> pwm_lidar(new MyRioPwm(MyRioPwm::PWMB2));	
	lidar_ = std::shared_ptr<Lidar>(new LidarA1(uart_B, pwm_lidar,LidarA1::LidarMod::k8k));

	//std::shared_ptr<Servo> servo_low(new Servo_ocs251(9,uart_B));
	//std::shared_ptr<Servo> servo_up(new Servo_ocs251(2, uart_B));
	//man_ = std::shared_ptr<Manipulator>(new Manipulator(servo_low,nullptr));
	std::shared_ptr<KangarooDriver> kangarooDriver1(new KangarooDriver(uart_A, 135));
	std::shared_ptr<KangarooDriver> kangarooDriver2(new KangarooDriver(uart_A, 130));
	std::shared_ptr<KangarooMotor> motor_front(new KangarooMotor(kangarooDriver1, '2', false));
	std::shared_ptr<KangarooMotor> motor_left(new KangarooMotor(kangarooDriver2, '1', true));
	std::shared_ptr<KangarooMotor> motor_back(new KangarooMotor(kangarooDriver2, '2', false));
	std::shared_ptr<KangarooMotor> motor_right(new KangarooMotor(kangarooDriver1, '1', true));
	omni_ = std::shared_ptr<OmniWheels4Squre>(new OmniWheels4Squre(50,
		150,
		motor_left,
		motor_front,
		motor_right,
		motor_back));
	
	indicator_ = std::shared_ptr<RgbLed>(new RgbLed(
		std::shared_ptr<MyRio_Aio>(new MyRio_Aio {AOA_0VAL, AOA_0WGHT, AOA_0OFST, AOSYSGO, NiFpga_False, 0.95,0}),//R
		std::shared_ptr<MyRio_Aio>(new MyRio_Aio {AOB_1VAL, AOB_1WGHT, AOB_1OFST, AOSYSGO, NiFpga_False, 0.85, 0}), //G
		std::shared_ptr<MyRio_Aio>(new MyRio_Aio {AOA_1VAL, AOA_1WGHT, AOA_1OFST, AOSYSGO, NiFpga_False, 0.85, 0})//B
		));
	dist_sensors_[DIST_LEFT] = std::shared_ptr<Sharp2_15>(
		new Sharp2_15(std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AIA_0VAL, AIA_0WGHT, AIA_0OFST, AOSYSGO, NiFpga_False, 1, 0 })));
	dist_sensors_[DIST_RIGHT]  = std::shared_ptr<Sharp2_15>(
		new Sharp2_15(std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AIA_3VAL, AIA_3WGHT, AIA_3OFST, AOSYSGO, NiFpga_False, 1, 0 })));
	dist_sensors_[DIST_C_LEFT]  = std::shared_ptr<Sharp2_15>(
		new Sharp2_15(std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AIA_1VAL, AIA_1WGHT, AIA_1OFST, AOSYSGO, NiFpga_False, 1, 0 })));
	dist_sensors_[DIST_C_RIGHT]  = std::shared_ptr<Sharp2_15>(
		new Sharp2_15(std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AIA_2VAL, AIA_2WGHT, AIA_2OFST, AOSYSGO, NiFpga_False, 1, 0 })));
}

RobotGardener::~RobotGardener()
{

	MyRio_Close();
}
Robot::~Robot()
{
}


std::shared_ptr<Indicator> RobotGardener::GetIndicator()
{
	return indicator_;
}





std::shared_ptr<DistanceSensor> RobotGardener::GetDistSensor(DistSensorEnum dist_sensor)
{
	return dist_sensors_[dist_sensor];
}


std::shared_ptr<Manipulator> RobotGardener::GetMan()
{
	return man_;
}


std::shared_ptr<Lidar> RobotGardener::GetLidar()
{
	return lidar_;
}


void RobotGardener::GetLidarPolarPoints(std::vector<PolarPoint>& polar_points)
{
	std::vector<LidarA1::Point> points_lidar;
	lidar_->GetScan(points_lidar);
	for (auto it = points_lidar.begin(); it != points_lidar.end(); ++it)
	{
		//TO DO filtering points
		polar_points.emplace_back(it->r, it->ph);
	}

}
