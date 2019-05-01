#include "Spi.h"
#include <memory>
extern NiFpga_Session myrio_session;
void SpiMyRio::Enable(int speed)
{
	EditSysSelect(0b11); // turn on spi
	Spi_Configure(&kMyRio_spi_regs_[(int)spi_port_],
		(Spi_ConfigureMask)((int)Spi_ClockPhase | (int)Spi_ClockPolarity | (int)Spi_DataOrder
                  | (int)Spi_FrameLength | (int)Spi_ClockDivider),
		(Spi_ConfigureSettings)((int)Spi_ClockPhaseLeading | (int)Spi_ClockPolarityLow
              | (int)Spi_DataOrderMsbFirst | (int)Spi_FrameSize8 | (int)Spi_Clock2x));
	SetSpeed(speed);
}


void SpiMyRio::SetSpeed(int speed)
{
	speed_ = speed;
	Spi_CounterMaximum(&kMyRio_spi_regs_[(int)spi_port_], (uint16_t)(kMyRioFreq_ / 2 / speed));
}


SpiMyRio::SpiMyRio(SpiPort port, 
	int speed)
	: spi_port_(port)
{
	Enable(speed);
}
void SpiMyRio::EditSysSelect(int bit)
{
	uint8_t selectReg;
	int status = NiFpga_ReadU8(myrio_session, kMyRio_sysselect_reg_[(int)spi_port_], &selectReg);
	if (MyRio_IsNotSuccess(status))
	{
		throw std::runtime_error("error spi enable");
	}
	selectReg = selectReg & (~(3));
	selectReg = selectReg | bit;
	
	status = NiFpga_WriteU8(myrio_session, kMyRio_sysselect_reg_[(int)spi_port_], selectReg);
	if (MyRio_IsNotSuccess(status))
	{
		throw std::runtime_error("error pwm enable");
	}
}

int SpiMyRio::GetSpeed()
{
	return speed_;
}


void SpiMyRio::Transmit(const uint8_t*data, uint8_t *read_data, size_t size_d)
{
	for (int i = 0; i < size_d; ++i)
	{
		uint16_t read_buf_;
		Spi_Transmit(&kMyRio_spi_regs_[(int)spi_port_], data[i], &read_buf_);
		read_data[i] = (uint8_t)read_buf_;
	}
}


uint8_t SpiMyRio::Transmit(uint8_t data)
{
	uint8_t read_buf_;
	Transmit(&data, &read_buf_, 1);
	return read_buf_;
}
