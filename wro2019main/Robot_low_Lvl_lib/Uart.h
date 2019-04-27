#pragma once
#include <vector>
#include "MyRio_lib\UART.h"
#include <memory>
class Uart
{
public:
	virtual int Send(const uint8_t* const data, const size_t nData) = 0;
	virtual int Send(const std::vector<uint8_t> &data) = 0;
	virtual void Clear() = 0;
	virtual int Get(uint8_t* const data,const size_t nData) = 0;
	virtual ~Uart(){}
	
};
class MyRioUart : public Uart
{
public:
	enum UartEnum
	{
		UART_A = 0,
		UART_B = 1
	};
	MyRioUart(UartEnum uart_enum,int speed);
	int  Send(const uint8_t* const data, const size_t nData) override;
	int Send(const std::vector<uint8_t> &data) override;
	int Get(uint8_t* const data, const size_t nData) override;
	void Clear() override;
	~MyRioUart();
private:
	std::shared_ptr<MyRio_Uart> uart_;
};