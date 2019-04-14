#include "Robot.h"
#include  <exception>

RobotGardener::RobotGardener()
{
	std::shared_ptr<MyRio_Uart> uart_A(new MyRio_Uart { "ASRL1::INSTR", 0, 0 });
	int status  = Uart_Open(uart_A.get(), 115200, 8, Uart_StopBits1_0, Uart_ParityNone);
	if (status != 0) throw std::runtime_error("Can't open uart A");
	std::shared_ptr<KangarooDriver> kangarooDriver1(new KangarooDriver(uart_A, 134));
	std::shared_ptr<KangarooDriver> kangarooDriver2(new KangarooDriver(uart_A, 135));
	std::shared_ptr<KangarooMotor> motor_front(new KangarooMotor(kangarooDriver1, 1));
	std::shared_ptr<KangarooMotor> motor_left(new KangarooMotor(kangarooDriver2, 2));
	std::shared_ptr<KangarooMotor> motor_back(new KangarooMotor(kangarooDriver1, 2));
	std::shared_ptr<KangarooMotor> motor_right(new KangarooMotor(kangarooDriver2, 1));
	omni = std::shared_ptr<OmniWheels4Squre>(new OmniWheels4Squre(30,
		250,
		motor_left,
		motor_front,
		motor_right,
		motor_back));
	
}


void RobotGardener::Start()
{
	omni->Move(std::make_pair(0, 300),0);
}
