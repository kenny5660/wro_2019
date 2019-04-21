#include "Robot.h"
#include  <exception>

RobotGardener::RobotGardener()
{

	
}

std::shared_ptr<OmniWheels> RobotGardener::GetOmni()
{
	return omni;
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
	std::shared_ptr<MyRio_Uart> uart_A(new MyRio_Uart { "ASRL1::INSTR", 0, 0 });
	int status  = Uart_Open(uart_A.get(), 115200, 8, Uart_StopBits1_0, Uart_ParityNone);
	if (status != 0) throw std::runtime_error("Can't open uart A");
	std::shared_ptr<KangarooDriver> kangarooDriver1(new KangarooDriver(uart_A, 135));
	std::shared_ptr<KangarooDriver> kangarooDriver2(new KangarooDriver(uart_A, 130));
	std::shared_ptr<KangarooMotor> motor_front(new KangarooMotor(kangarooDriver1, '2', false));
	std::shared_ptr<KangarooMotor> motor_left(new KangarooMotor(kangarooDriver2, '1', true));
	std::shared_ptr<KangarooMotor> motor_back(new KangarooMotor(kangarooDriver2, '2', false));
	std::shared_ptr<KangarooMotor> motor_right(new KangarooMotor(kangarooDriver1, '1', true));
	omni = std::shared_ptr<OmniWheels4Squre>(new OmniWheels4Squre(50,
		150,
		motor_left,
		motor_front,
		motor_right,
		motor_back));
}


Robot::~Robot()
{
}
