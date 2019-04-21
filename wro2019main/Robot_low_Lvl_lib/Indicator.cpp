#include "Indicator.h"


void RgbLed::Display(DisplayEnum disp)
{
	RgbLedVal rgb_val = rgb_led_vals_[disp];
	Aio_Write_uint16_t(ao_R_.get(), rgb_val.R * 16);
	Aio_Write_uint16_t(ao_B_.get(), rgb_val.B * 16);
	Aio_Write_uint16_t(ao_G_.get(), rgb_val.G * 16);
}

RgbLed::RgbLed(std::shared_ptr<MyRio_Aio>  ao_R, 
	std::shared_ptr<MyRio_Aio>  ao_G_, 
	std::shared_ptr<MyRio_Aio>  ao_B_)
	: ao_R_(ao_R)
	, ao_G_(ao_G_)
	, ao_B_(ao_B_)
{
}
