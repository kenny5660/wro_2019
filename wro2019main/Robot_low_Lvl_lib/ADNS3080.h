#pragma once
#include "Spi.h"
#include "GPIO.h"
#include <utility>
#include <memory>
#include <atomic>
#include <thread>
class ADNS3080 : public  OpticalFlow
{
public:
	ADNS3080(std::shared_ptr<Spi> spi, std::shared_ptr<GPIO> ss_pin);
	std::pair<double, double> GetPos() override;
	void Reset() override;	
private:
	bool Ping_();
	int ReadReg(uint8_t reg, uint8_t* data, size_t size);
	int WriteReg(uint8_t reg, uint8_t* data, size_t size);
	std::atomic_int_fast32_t pos_x_;
	std::atomic_int_fast32_t pos_y_;
	std::shared_ptr<Spi> spi_;
	std::shared_ptr<GPIO> ss_pin_;
	
	const uint8_t kProductId_ = 0x17;
	enum class ADNS3080_REG
	{
		PRODUCT_ID = 0x00,
		MOTION = 0x02,
		DELTA_X = 0x03,
		DELTA_Y = 0x04,
		SQUAL = 0x05,
		CONFIGURATION_BITS = 0x0A,
		MOTION_CLEAR = 0x12,
		FRAME_CAPTURE = 0x13,
		MOTION_BURST = 0x50,
		// Id returned by ADNS3080_PRODUCT_ID register
	};
	virtual ~ADNS3080()
	{ 
	}
};