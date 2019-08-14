#include "Robot.h"
#include "RobotHardwere.h"
#include "Uart.h"
#include "Pwm.h"
#include "lidar_math.h"
#include  <exception>
#include <csignal>
#include <chrono>
#include "debug.h"
#include "VisionAlgs.h"
#include <algorithm>
extern NiFpga_Session myrio_session;

RobotGardenerHardwere::RobotGardenerHardwere()
{
	NiFpga_Status status_rio = MyRio_Open();
	if (MyRio_IsNotSuccess(status_rio)) {
		throw std::runtime_error("Error open MyRio!");
	}
	
}
void RobotGardenerHardwere::Init()
{

	indicator_ = std::shared_ptr<RgbLed>(new RgbLed(
				//R
	std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AOA_0VAL, AOA_0WGHT, AOA_0OFST, AOSYSGO, NiFpga_False, 0.95, 0 }),
		//G
	std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AOB_1VAL, AOB_1WGHT, AOB_1OFST, AOSYSGO, NiFpga_False, 0.85, 0 }),
		//B
	std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AOA_1VAL, AOA_1WGHT, AOA_1OFST, AOSYSGO, NiFpga_False, 0.85, 0 })));
	indicator_->Display(Indicator::YELLOW);
	Delay(1000);
	start_but_ = std::make_shared<ButtonOnMyrio>();
	std::shared_ptr<Uart> uart_A(new MyRioUart(MyRioUart::UART_A, 115200));
	std::shared_ptr<Uart> uart_B(new MyRioUart(MyRioUart::UART_B, 115200));
	std::shared_ptr<Spi> spi_A(std::make_shared<SpiMyRio>(SpiMyRio::SPIA, SpiMyRio::SpiSpeed::kSpeed02Mbit));
	std::shared_ptr<Uart> uart_Bridge(std::make_shared<UartSc16is750>(spi_A, std::make_shared<GPIOmyRio>(GPIOmyRio::PortMyRio::A, 4), 115200));
	
	uart_A->Clear();
	uart_B->Clear();
	uart_Bridge->Clear();
	std::shared_ptr<Servo> servo_cam(new ServoOcs251(8, uart_Bridge));
	std::shared_ptr<Servo> servo_up(new ServoOcs251(5, uart_Bridge));
	std::shared_ptr<Servo> servo_low(new ServoOcs301(3, uart_Bridge));
	

	
	

	cam_rot_ = std::make_shared<CameraRotate>(0, servo_cam);
	cam_rot_->SetResolution(std::make_pair(1024, 768));
	man_ = std::shared_ptr<Manipulator>(new Manipulator(servo_low, servo_up));

	std::shared_ptr<KangarooDriver> kangarooDriver2(new KangarooDriver(uart_A, 130));
	std::shared_ptr<KangarooDriver> kangarooDriver1(new KangarooDriver(uart_A, 135));
	std::shared_ptr<MotorKangaroo> motor_front(new MotorKangaroo(kangarooDriver2, '1', true));
	std::shared_ptr<MotorKangaroo> motor_left(new MotorKangaroo(kangarooDriver2, '2', false));
	std::shared_ptr<MotorKangaroo> motor_back(new MotorKangaroo(kangarooDriver1, '1', true));
	std::shared_ptr<MotorKangaroo> motor_right(new MotorKangaroo(kangarooDriver1, '2', false));
	omni_ = std::shared_ptr<OmniWheels4Squre>(new OmniWheels4Squre(50,
		131,
		motor_left,
		motor_front,
		motor_right,
		motor_back));
	

	const int dist_sensor_filter_win_size = 10;
	dist_sensors_[DIST_LEFT] = std::make_shared<Sharp2_15>(
		std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AIA_0VAL, AIA_0WGHT, AIA_0OFST, AOSYSGO, NiFpga_False, 1, 0 }), dist_sensor_filter_win_size);
	dist_sensors_[DIST_TOP]  = std::make_shared<Sharp2_15>(
		std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AIA_3VAL, AIA_3WGHT, AIA_3OFST, AOSYSGO, NiFpga_False, 1, 0 }), dist_sensor_filter_win_size);
	dist_sensors_[DIST_C_LEFT]  = std::make_shared<Sharp2_15>(
		std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AIA_1VAL, AIA_1WGHT, AIA_1OFST, AOSYSGO, NiFpga_False, 1, 0 }), dist_sensor_filter_win_size);
	dist_sensors_[DIST_C_RIGHT]  = std::make_shared<Sharp2_15>(
		std::shared_ptr<MyRio_Aio>(new MyRio_Aio { AIA_2VAL, AIA_2WGHT, AIA_2OFST, AOSYSGO, NiFpga_False, 1, 0 }), dist_sensor_filter_win_size);
	opt_flow_ = std::make_shared<HidMice>("/dev/input/mouse0", std::make_pair(0.053326945, 0.053326945), -90);               //0.018,-90);
	
		std::shared_ptr<Pwm> pwm_lidar(new PwmMyRio(PwmMyRio::PWMB2));	
	lidar_ = std::shared_ptr<Lidar>(new LidarA1(uart_B, pwm_lidar, LidarA1::LidarMod::k8k));
	lidar_->StartScan(0.2);
	
	man_->CatchRight();
	man_->Home();
	indicator_->Display(Indicator::WHITE);
	Delay(400);
	indicator_->Display(Indicator::OFF);
	Delay(400);
	indicator_->Display(Indicator::WHITE);
	// Install a signal handler
	std::cout << "Robot init done" << std::endl;
	WaitStartButton();
	indicator_->Display(Indicator::OFF);
}

RobotGardenerHardwere::~RobotGardenerHardwere()
{
	indicator_->Display(Indicator::OFF);
}
void RobotGardenerHardwere::GetLidarPolarPoints(std::vector<PolarPoint>& polar_points)
{
	const double kLidarDegOffset = -48.9;   // 45;
	polar_points.clear();
	std::vector<LidarA1::Point> points_lidar;
	lidar_->GetScan(points_lidar);
	points_lidar.clear();
	Delay(1111);
	lidar_->GetScan(points_lidar);
	
	for (auto it = points_lidar.begin(); it != points_lidar.end(); ++it)
	{
		//TO DO filtering points
		
		polar_points.emplace_back(it->r, it->ph + (kLidarDegOffset*M_PI / 180));
	}
	struct sort_class_PolarPoint
	{
		bool operator()(PolarPoint i, PolarPoint j)
		{ return (i.get_f()  < j.get_f());}
	} sort_object_PolarPoint;
	std::sort(polar_points.begin(), polar_points.end(), sort_object_PolarPoint);
	data_filter(polar_points);
	save_ld_data(polar_points);
}