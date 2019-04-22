#include "Robot.h"
#include  <exception>
extern NiFpga_Session myrio_session;
RobotGardener::RobotGardener()
{

	
}

std::shared_ptr<OmniWheels> RobotGardener::GetOmni()
{
	return omni_;
}

void Robot::Init()
{throw std::runtime_error("Not implemented");}
void Robot::Delay(int msec)
{
	struct timespec tw = { msec / 1000, (msec % 1000) * 1000000 };
	struct timespec tr;
	nanosleep(&tw, &tr);
}


void RobotGardener::Init()
{
	NiFpga_Status status_rio = MyRio_Open();
	if (MyRio_IsNotSuccess(status_rio)) {
		throw std::runtime_error("Error open MyRio!");
	}
	std::shared_ptr<MyRio_Uart> uart_A(new MyRio_Uart { "ASRL1::INSTR", 0, 0 });
	int status  = Uart_Open(uart_A.get(), 115200, 8, Uart_StopBits1_0, Uart_ParityNone);
	if (status != 0) throw std::runtime_error("Can't open uart A");
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
	
}


Robot::~Robot()
{
}


std::shared_ptr<Indicator> RobotGardener::GetIndicator()
{
	return indicator_;
}


RobotGardener::~RobotGardener()
{
	MyRio_Close();
}
