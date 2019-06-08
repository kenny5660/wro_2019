#pragma once
#include <utility>
#include <memory>
#include <atomic>
#include <thread>
#include "Spi.h"
#include "GPIO.h"
class OpticalFlow
{
public:
	virtual std::pair<double, double> GetPos() = 0;
	virtual void Reset() = 0;
	virtual ~OpticalFlow() {}
};
class HidMice : public  OpticalFlow
{
public:
	HidMice(std::string pDevice, double mice_to_mm_coef, double coord_angle = 0);
	std::pair<double, double> GetPos() override;
	void Reset() override;
	virtual ~HidMice();
	
private:
	void Start();
	void ThreadReadMice_Pos_(int fd);
	std::shared_ptr<std::thread> thread_read_mice_;
	std::atomic_bool  thread_stop;
	std::string pDevice_;
	std::atomic_int_fast32_t pos_x_;
	std::atomic_int_fast32_t pos_y_;
	double mice_to_mm_coef_;
	int fd_;
	double coord_angle_;
};
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