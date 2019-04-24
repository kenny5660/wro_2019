#include "Uart.h"
#include <string>






MyRioUart::MyRioUart(UartEnum uart_enum,int speed)
{
	switch (uart_enum)
	{
	case UART_A : uart_ = std::shared_ptr<MyRio_Uart>(new MyRio_Uart { "ASRL1::INSTR", 0, 0 }); break;
	case UART_B : uart_ = std::shared_ptr<MyRio_Uart>(new MyRio_Uart { "ASRL2::INSTR", 0, 0 }); break;
	default:
		break;
	}
	int status  = Uart_Open(uart_.get(), 115200, 8, Uart_StopBits1_0, Uart_ParityNone);
	if (status != 0) throw std::runtime_error((std::string)"Can't open uart" + (uart_enum  == UART_A ? "A":"B"));
}

int MyRioUart::Send(const uint8_t* const data, const size_t nData)
{
	return Uart_Write(uart_.get(), data, nData);
}

int MyRioUart::Send(const std::vector<uint8_t> &data)
{
	return Uart_Write(uart_.get(), data.data(), data.size());
}

int MyRioUart::Get(uint8_t* const data, 
	const size_t nData)
{
	return Uart_Read(uart_.get(), data, nData);
}


MyRioUart::~MyRioUart()
{
	Uart_Close(uart_.get());
}


void MyRioUart::Clear()
{
	Uart_Clear(uart_.get());
}
