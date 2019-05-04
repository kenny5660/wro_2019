#include <GPIO.h>
#include <exception>
const MyRio_Dio kMyRioDioReg[5] = { 
	{DIOA_70DIR,DIOA_70OUT,DIOA_70IN,0},
	{DIOB_70DIR, DIOB_70OUT, DIOB_70IN, 0},
	{DIOC_70DIR, DIOC_70OUT, DIOC_70IN, 0},
	{DIOA_158DIR, DIOA_158OUT, DIOA_158IN, 0},
    {DIOB_158DIR, DIOB_158OUT, DIOB_158IN, 0}
	};


GPIOmyRio::GPIOmyRio(PortMyRio port, 
	uint8_t pin) : port_(port),pin_(pin)
{
	if (port_ == PortMyRio::C && pin_ > 7)
	{
		throw std::runtime_error("Error, Port C have only DIO 0 - DIO 7");
	}
	if (pin_ > 15)
	{
		throw std::runtime_error("Error, Port have only DIO 0 - DIO 15");
	}
	
}

void GPIOmyRio::Set(uint8_t bit)
{
	MyRio_Dio dio;
	DioRegsSet(&dio);
	Dio_WriteBit(&dio, bit);
}

void GPIOmyRio::DioRegsSet(MyRio_Dio* dio)
{
	if (pin_ > 7)
	{
		*dio = kMyRioDioReg[(int)port_ + 3];
		dio->bit = pin_ - 8;
	}
	else
	{
		*dio = kMyRioDioReg[(int)port_];
		dio->bit = pin_;
	}
}


uint8_t GPIOmyRio::Get()
{
	MyRio_Dio dio;
	DioRegsSet(&dio);
	return Dio_ReadBit(&dio);
}


void GPIOmyRio::Set()
{
	Set(1);
}


void GPIOmyRio::Reset()
{
	Set(0);
}
