#include "Uart.h"
#include <string>
#include <chrono>
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

int Uart::Send(const std::vector<uint8_t> &data)
{
    Send(data.data(), data.size());
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


std::string MyRioUart::GetFilePath()
{
	return uart_enum_ == MyRioUart::UART_A ? "/dev/ttyS0" : "/dev/ttyS1";
}


UartSc16is750::UartSc16is750(std::shared_ptr<Spi> spi, 
	std::shared_ptr<GPIO> ss_pin, 
	int speed)
	: sc16is750_(std::make_shared<SC16IS750>(spi, ss_pin))
{
	sc16is750_->begin(speed);
}


UartSc16is750::UartSc16is750(std::shared_ptr<SC16IS750> sc16is750, 
	int speed)
	: sc16is750_(sc16is750)
{
	sc16is750_->begin(speed);
}



std::shared_ptr<SC16IS750> UartSc16is750::GetSc16is750()
{
	return sc16is750_;
}


void UartSc16is750::SetBaundRate(int baund)
{
	sc16is750_->begin(baund);
}


int UartSc16is750::Send(const uint8_t* const data, const size_t nData)
{
	for (int i = 0; i < nData; ++i)
	{
		sc16is750_->write(data[i]);
	}
	return nData;
}


int UartSc16is750::Get(uint8_t* const data, const size_t nData)
{
	using namespace std::chrono;
	const milliseconds timeOut(1000);
	for (int i = 0; i < nData; ++i)
	{
		steady_clock::time_point startTime = steady_clock::now(); 
		while (!sc16is750_->available())
		{
			if (steady_clock::now() - startTime  > timeOut)
			{
				cur_status_ = ErrorStatus::TIMEOUT;
				break;
			}
		}
		if (isError())
		{
			break;
		}
		data[i] = sc16is750_->read();
	}
	return nData;
}


std::string UartSc16is750::GetFilePath()
{
	std::runtime_error("Error,UartSc16is750 have not file path!");
	return "";
}


void UartSc16is750::Clear()
{
	sc16is750_->flush();
}

bool UartSc16is750::isError()
{
	return cur_status_ != ErrorStatus::NORMAL;
}
