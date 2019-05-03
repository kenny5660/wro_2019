#pragma once
#include <memory>
#include <DIO.h>
class GPIO
{
public:
	virtual void Set() = 0;
	virtual void Set(uint8_t bit) = 0;
	virtual void Reset() = 0;
	virtual uint8_t Get() = 0;
};

class GPIOmyRio : public GPIO
{
public:
	enum class PortMyRio
	{
		A = 0,
		B = 1,
		C = 2
	};
	GPIOmyRio(PortMyRio port, uint8_t pin);
	virtual void Set();
	virtual void Set(uint8_t bit);
	virtual void Reset();
	virtual uint8_t Get();
private:
	void DioRegsSet(MyRio_Dio* dio);
	PortMyRio port_;
	uint8_t pin_;
};