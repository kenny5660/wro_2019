#pragma once
#include <vector>
#include "MyRio_lib\UARTRio.h"
#include <memory>
#include <string>
#include "Spi.h"
#include "GPIO.h"
#include "SC16IS750/SC16IS750.h"
class Uart
{
public:
	virtual void SetBaundRate(int baund) = 0;
	virtual int Send(const uint8_t* const data, const size_t nData) = 0;
	virtual int Send(const std::vector<uint8_t> &data);
	virtual void Clear() = 0;
	virtual int Get(uint8_t* const data,const size_t nData) = 0;
	virtual bool isError() = 0;
	virtual std::string GetFilePath() = 0;
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
	void SetBaundRate(int baund) override;
	int  Send(const uint8_t* const data, const size_t nData) override;
	int Get(uint8_t* const data, const size_t nData) override;
	std::string GetFilePath() override;
	void Clear() override;
	bool isError() override;
	~MyRioUart();
private:
	std::shared_ptr<MyRio_Uart> uart_;
	UartEnum uart_enum_;
	int cur_status_ = 0;
	
};
class UartSc16is750 : public Uart
{
public:
	UartSc16is750(std::shared_ptr<Spi> spi, std::shared_ptr<GPIO> ss_pin, int speed);
	UartSc16is750(std::shared_ptr<SC16IS750> sc16is750, int speed);
	std::shared_ptr<SC16IS750> GetSc16is750();
	void SetBaundRate(int baund) override;
	int  Send(const uint8_t* const data, const size_t nData) override;
	int Get(uint8_t* const data, const size_t nData) override;
	std::string GetFilePath() override;
	void Clear() override;
	bool isError() override;
private:
	enum class ErrorStatus
	{
		NORMAL = 0,
		TIMEOUT = 1,
		NOTCONNECTED = 2
	};
	std::shared_ptr<SC16IS750> sc16is750_;
	ErrorStatus cur_status_ = ErrorStatus::NORMAL;
};