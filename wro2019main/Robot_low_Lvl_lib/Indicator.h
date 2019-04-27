#pragma once
#include "MyRio_lib/AIO.h"
#include <utility>
#include <memory>
#include <vector>
class Indicator
{
public:
	enum DisplayEnum
	{
		ERROR  = 0,
		GREEN,
		RED,
		D_BLUE,
		ORANGE,
		WHITE,
		OFF
	};

	virtual void Display(DisplayEnum disp);
	virtual ~Indicator() {}

};

class RgbLed : public Indicator
{

public:
	RgbLed(std::shared_ptr<MyRio_Aio>  ao_R,
		std::shared_ptr<MyRio_Aio>  ao_G_,
		std::shared_ptr<MyRio_Aio>  ao_B_);
	void Display(DisplayEnum disp) override;
private:
	struct  RgbLedVal {
		uint16_t R;
		uint16_t G;
		uint16_t B;
	};
	
	std::shared_ptr<MyRio_Aio>  ao_R_;
	std::shared_ptr<MyRio_Aio>  ao_G_;
	std::shared_ptr<MyRio_Aio>  ao_B_;
	const std::vector<RgbLedVal> rgb_led_vals_ = 
		{ /*ERROR*/{ 255, 0, 0 }, /*GREEN*/{ 0, 255, 0 }, /*RED*/{ 255, 0, 0 },
		/*D_BLUE*/{ 0, 0, 255 }, /*ORANGE*/{ 255, 255, 0 }, /*WHITE*/{ 255,255,255 }, /*OFF*/{ 0, 0, 0 } };
};
