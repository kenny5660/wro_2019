#pragma once
#include <memory>
#include <vector>
#include <utility>
#include "MyRio_lib/AIO.h"
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
	{
		/*ERRORR*/{ 255, 0, 0 },
		/*GREEN*/{ 0, 255, 0 },
		/*RED*/{ 255, 0, 0 },
		/*D_BLUE*/{ 0, 0, 255 },
		/*ORANGE*/{ 255, 200, 0 },
		/*WHITE*/{ 255, 255, 255 },
		/*OFF*/{ 0, 0, 0 },
		/*YELLOW*/{ 211, 190, 13 }
	 };
};