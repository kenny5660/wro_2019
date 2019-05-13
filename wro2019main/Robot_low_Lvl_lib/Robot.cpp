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
	//	struct timespec tw = { msec / 1000, (msec % 1000) * 1000000 };
	//	struct timespec tr;
	//	nanosleep(&tw, &tr);
		std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}


void RobotGardener::Init()
{
	
	std::shared_ptr<Uart> uart_A(new MyRioUart(MyRioUart::UART_A, 115200));
	std::shared_ptr<Uart> uart_B(new MyRioUart(MyRioUart::UART_B, 115200));
	std::shared_ptr<Spi> spi_A(std::make_shared<SpiMyRio>(SpiMyRio::SPIA, SpiMyRio::SpiSpeed::kSpeed05Mbit));
	
	std::shared_ptr<Uart> uart_Bridge(std::make_shared<UartSc16is750>(spi_A, std::make_shared<GPIOmyRio>(GPIOmyRio::PortMyRio::A, 4), 115200));
	
	std::shared_ptr<Pwm> pwm_lidar(new PwmMyRio(PwmMyRio::PWMB2));	
	lidar_ = std::shared_ptr<Lidar>(new LidarA1(uart_B, pwm_lidar,LidarA1::LidarMod::k8k));
	lidar_->StartScan(0.4);
	std::shared_ptr<Servo> servo_low(new Servo_ocs251(9, uart_Bridge));
	std::shared_ptr<Servo> servo_up(new Servo_ocs251(5, uart_Bridge));
	std::shared_ptr<Servo> servo_cam(new Servo_ocs251(8, uart_Bridge));
	cam_rot_ = std::make_shared<CameraRotate>(0, servo_cam);
	cam_rot_->SetResolution(std::make_pair(1024, 768));
	man_ = std::shared_ptr<Manipulator>(new Manipulator(servo_low, servo_up));
	std::shared_ptr<KangarooDriver> kangarooDriver1(new KangarooDriver(uart_A, 135));
	std::shared_ptr<KangarooDriver> kangarooDriver2(new KangarooDriver(uart_A, 130));
	std::shared_ptr<KangarooMotor> motor_front(new KangarooMotor(kangarooDriver2, '1', true));
	std::shared_ptr<KangarooMotor> motor_left(new KangarooMotor(kangarooDriver2, '2', false));
	std::shared_ptr<KangarooMotor> motor_back(new KangarooMotor(kangarooDriver1, '1', true));
	std::shared_ptr<KangarooMotor> motor_right(new KangarooMotor(kangarooDriver1, '2', false));
	omni_ = std::shared_ptr<OmniWheels4Squre>(new OmniWheels4Squre(48,
		115,
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
	man_->CatchRight();
	man_->Home();
}

RobotGardener::~RobotGardener()
{
 

}
Robot::~Robot()
{
	MyRio_Close();
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


void RobotGardener::CatchCube()
{
	const int kDist = 60;
	const int kOfsetAngle = 0;
	const int speed = 200;
	man_->CatchRight();
	AlliginByDist(kDist, kOfsetAngle);
	std::cout << "Dist left = " <<  GetDistSensor(DIST_LEFT)->GetDistance()<< std::endl;
	AlliginRight();
	std::cout << "Dist left = " <<  GetDistSensor(DIST_LEFT)->GetDistance() << std::endl;
	man_->Out(true);
	omni_->MoveToPosInc(std::make_pair(0, 80), speed);
	man_->CatchLeft(true);
	omni_->MoveToPosInc(std::make_pair(0, 80), speed);
	man_->Home();
	
}


void RobotGardener::AlliginByDist(int dist,int offset_alg)
{
	std::shared_ptr<DistanceSensor> dist_left = GetDistSensor(DIST_C_LEFT);
	std::shared_ptr<DistanceSensor> dist_right = GetDistSensor(DIST_C_RIGHT);
	using namespace std::chrono;
	const milliseconds timeOut(100);
	const double P_align = 5;
	const double D_aligin = 8;
	const double P_dist = -8;
	const double D_dist = 4;
	int alg_speed = 0;
	int x_speed = 0;
	int err_dist = INT_MAX;
	int err_dist_old = 0;
	int err_align_old = 0;
	int err_align  = INT_MAX;
	steady_clock::time_point startTime = steady_clock::now(); 
	
	while ((steady_clock::now() - startTime)  < timeOut)//(abs(err_align) > 2  || abs(err_dist) > 2)
	{
		err_align = dist_left->GetDistance() - dist_right->GetDistance() + offset_alg;
		alg_speed = err_align*P_align + D_aligin*(err_align - err_align_old);
		
		err_dist = dist - dist_left->GetDistance();
		x_speed = err_dist*P_dist + D_dist*(err_dist - err_dist_old);
		err_dist_old = err_dist;
		err_align_old = err_align;
		GetOmni()->MoveWithSpeed(std::make_pair(x_speed,0), alg_speed);
		Delay(5);
		if ((abs(err_align) > 4 || abs(err_dist) > 4))
		{
			 startTime = steady_clock::now(); 
		}
		
	}
	
	GetOmni()->Stop();
}


void RobotGardener::AlliginRight()
{
	const int mid_dist = 200;
	const int speed = 100;
	std::shared_ptr<DistanceSensor> dist = GetDistSensor(DIST_LEFT);

	if (dist->GetDistance() > mid_dist)
	{
		GetOmni()->MoveWithSpeed(std::make_pair(0, -speed), 0);
		while (dist->GetDistance() > mid_dist);
	}
	else
	{
		GetOmni()->MoveWithSpeed(std::make_pair(0,speed), 0);
		while (dist->GetDistance() < mid_dist) ;
	}
	GetOmni()->Stop();
}


std::shared_ptr<CameraRotate> RobotGardener::GetCamRot()
{
	return cam_rot_;
}


std::shared_ptr<cv::Mat> RobotGardener::GetQrCodeFrame()
{
	const int kDegServo = 268;
	const int kmidDist  = 200;
	std::shared_ptr<DistanceSensor> dist_sensor = GetDistSensor(DIST_LEFT);
	omni_->MoveWithSpeed(std::make_pair(0, 250),0);
	while (dist_sensor->GetDistance() < kmidDist);
	omni_->Stop();
	Delay(300);
	return cam_rot_->GetFrame(kDegServo);
}
