#include "DistanceSensor.h"
#include <cmath>
#include <thread>

int Sharp2_15::GetDistance()
{
	uint16_t analog = Aio_Read(analog_in_.get());
	int dist = kPowerCoeffC*pow(analog >> 2,kPowerCoeffP); 
	std::this_thread::sleep_for(std::chrono::microseconds(100));
	return  m_filter_.in(dist); 
} 
void Sharp2_15::SkipWindow()
{
	for (int i = 0; i < m_filter_.getWindowSize()*2; ++i)
	{
		GetDistance();
	}
}
Sharp2_15::Sharp2_15(std::shared_ptr<MyRio_Aio> anlog_in, int median_filter_w_size, int median_filter_seed)
	: analog_in_(anlog_in)
	, m_filter_(median_filter_w_size, median_filter_seed)
{
}


int Sharp2_15::GetRealDistance()
{
	SkipWindow();
	return GetDistance();
}
