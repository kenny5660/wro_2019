#include "Robot.h"
#include "Uart.h"
#include "PWM.h"
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
	//std::shared_ptr<Uart> uart_B(new MyRioUart(MyRioUart::UART_B, 115200));
	std::shared_ptr<MyRio_Dio> dtr_pin(new MyRio_Dio{DIOB_158DIR, DIOB_158OUT, DIOB_158IN,2});
	
	
	
	
	uint8_t selectReg;
	std::shared_ptr<MyRio_Pwm> pwm_lidar(new MyRio_Pwm{PWMB_2CNFG, PWMA_0CS, PWMA_0MAX, PWMA_0CMP, PWMA_0CNTR});
	Pwm_Configure(pwm_lidar.get(),
		Pwm_Invert | Pwm_Mode,
		Pwm_NotInverted | Pwm_Enabled);
	Pwm_ClockSelect(pwm_lidar.get(), Pwm_64x);
	Pwm_CounterMaximum(pwm_lidar.get(), 1000);
	Pwm_CounterCompare(pwm_lidar.get(), 400);
	int status = NiFpga_ReadU8(myrio_session, SYSSELECTB, &selectReg);

	/*
	 * Set bit2 of the SYSSELECTA register to enable PWMA_0 functionality.
	 * The functionality of the bit is specified in the documentation.
	 */

	selectReg = selectReg | (1 << 4);

	/*
	 * Write the updated value of the SYSSELECTA register.
	 */
	status = NiFpga_WriteU8(myrio_session, SYSSELECTB, selectReg);
	
		
		
		
	//lidar = std::shared_ptr<rp::standalone::rplidar::RPlidarDriver>(rp::standalone::rplidar::RPlidarDriver::CreateDriver(uart_B,dtr_pin));
	
	lidar = std::shared_ptr<rp::standalone::rplidar::RPlidarDriver>(rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT));
	lidar->connect("/dev/ttyS1", 115200);
	lidar->startMotor();
	std::vector<rp::standalone::rplidar::RplidarScanMode> scanModes;
	rp::standalone::rplidar::RplidarScanMode scanMode;
	int res  = lidar->getAllSupportedScanModes(scanModes);
	if (IS_FAIL(res))
	{
		throw std::runtime_error("Lidar connection error!");
	}
	lidar->startScanExpress(false, scanModes[(int)kLidar_mode_].id, 0, &scanMode);
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
	lidar->disconnect();
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


std::shared_ptr<rp::standalone::rplidar::RPlidarDriver> RobotGardener::GetLidar()
{
	return lidar;
}


void RobotGardener::GetLidarPolarPoints(std::vector<PolarPoint>& polar_points)
{
	int res;
	rplidar_response_measurement_node_hq_t nodes[kLidarPoints_[(int)kLidar_mode_]];
	size_t size_nodes = kLidarPoints_[(int)kLidar_mode_];
	do
	{
		res = lidar->grabScanDataHq(nodes, size_nodes);
	} while (IS_FAIL(res));
	lidar->ascendScanData(nodes, size_nodes);
	
	for (int i = 0; i  < size_nodes;++i)
	{
		float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
		float distance_in_mm = nodes[i].dist_mm_q2 / (1 << 2);
		PolarPoint pp(distance_in_mm, (double)angle_in_degrees / 180 * M_PI);
		polar_points.push_back(pp);
	}

}
