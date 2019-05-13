#include "DistanceSensor.h"
#include <cmath>
#include <thread>
int Sharp2_15::GetDistance()
{
	uint16_t analog = Aio_Read(analog_in_.get());
	int dist = kPowerCoeffC*pow(analog >> 2,kPowerCoeffP); 
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	return dist; 
	
} 

Sharp2_15::Sharp2_15(std::shared_ptr<MyRio_Aio> anlog_in)
	:analog_in_(anlog_in)
{
}
