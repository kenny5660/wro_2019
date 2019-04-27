#include "Uart.h"
#include <string>

MyRioUart::MyRioUart(UartEnum uart_enum, int speed)
	: uart_enum_(uart_enum)
{
	switch (uart_enum_)
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
	cur_status_ =  Uart_Write(uart_.get(), data, nData);
	return nData;
}

int MyRioUart::Send(const std::vector<uint8_t> &data)
{
	cur_status_ =   Uart_Write(uart_.get(), data.data(), data.size());
	return data.size();
}

int MyRioUart::Get(uint8_t* const data, 
	const size_t nData)
{
	cur_status_ =  Uart_Read(uart_.get(), data, nData);
	return nData;
}


MyRioUart::~MyRioUart()
{
	Uart_Close(uart_.get());
}


void MyRioUart::Clear()
{
	Uart_Clear(uart_.get());
}


void MyRioUart::SetBaundRate(int baund)
{
	Uart_Close(uart_.get());
	int status  = Uart_Open(uart_.get(), baund, 8, Uart_StopBits1_0, Uart_ParityNone);
	if (status != 0) throw std::runtime_error((std::string)"Can't setBaundRate" + uart_->name);

}


bool MyRioUart::isError()
{
	return cur_status_ < VI_SUCCESS;
}
