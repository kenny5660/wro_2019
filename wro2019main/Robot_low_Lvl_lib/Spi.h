#pragma once
#include "SPImyRio.h"
class Spi
{
public:
	virtual void Transmit(const uint8_t*data, uint8_t *read_data, size_t size_d) = 0;
	virtual uint8_t Transmit(uint8_t data) = 0;
	//@brief Set speed
    //@param speed in Hz(bit/sec)
	virtual void SetSpeed(int speed) = 0;
	virtual int GetSpeed() = 0;
	virtual ~Spi() = 0;
};

class SpiMyRio : public Spi
{
public:
	enum SpiPort
	{
		SPIA = 0,
		SPIB = 1
	};
	enum class SpiSpeed
	{
		 kSpeed4Mbit = 4000000,
		 kSpeed2Mbit = 2000000,
		 kSpeed1Mbit = 1000000,
		 kSpeed05Mbit = 500000
	};
	//@brief Create and Enable spi
    //@param speed in  Hz(bit/sec) 310 min 20 000 000 max
	SpiMyRio(SpiPort port, int speed);
	void Transmit(const uint8_t*data, uint8_t *read_data, size_t size_d) override;
	uint8_t Transmit(uint8_t data) override;
	//@brief Set speed
    //@param speed in Hz(bit/sec)
	void SetSpeed(int speed) override;
	int GetSpeed() override;
	virtual ~SpiMyRio();

private:
	SpiPort spi_port_;
	void Enable(int speed);
	void Disable();
	void EditSysSelect(int bit);
	int speed_;
};