#include "Pwm.h"
#include "MyRio_lib/PWM.h"
#include <memory>
extern NiFpga_Session myrio_session;
const MyRio_Pwm kMyRio_pwm_regs_[8] = {
	{ PWMA_0CNFG, PWMA_0CS, PWMA_0MAX, PWMA_0CMP, PWMA_0CNTR },
	{ PWMA_1CNFG, PWMA_1CS, PWMA_1MAX, PWMA_1CMP, PWMA_1CNTR },
	{ PWMA_2CNFG, PWMA_2CS, PWMA_2MAX, PWMA_2CMP, PWMA_2CNTR },
	{ PWMB_0CNFG, PWMB_0CS, PWMB_0MAX, PWMB_0CMP, PWMB_0CNTR },
	{ PWMB_1CNFG, PWMB_1CS, PWMB_1MAX, PWMB_1CMP, PWMB_1CNTR },
	{ PWMB_2CNFG, PWMB_2CS, PWMB_2MAX, PWMB_2CMP, PWMB_2CNTR },
	{ PWMC_0CNFG, PWMC_0CS, PWMC_0MAX, PWMC_0CMP, PWMC_0CNTR },
	{ PWMC_1CNFG, PWMC_1CS, PWMC_1MAX, PWMC_1CMP, PWMC_1CNTR }
}; 
const uint32_t kSysselct_reg[8] = {
	SYSSELECTA,
	SYSSELECTA,
	SYSSELECTA,
	SYSSELECTB,
	SYSSELECTB,
	SYSSELECTB,
	SYSSELECTC,
	SYSSELECTC, 
};
const uint32_t kSysselct_reg_bit[8] = {
	2,
	3,
	4,
	2,
	3,
	4,
	1,
	3, 
};
const int kMyRioFreq_ = 40000000;    //Hz 
const Pwm_ClockDivider kClkDiv_ = Pwm_8x;
const int kClkDivides_[8] = { 0, 1, 2, 4, 8, 16, 32, 64 };

PwmMyRio::PwmMyRio(PwmPin pwmPin)
	: pwmPin_(pwmPin)
{
}

PwmMyRio::PwmMyRio(PwmPin pwmPin, 
	int freq, 
	double duty /* = 0 */)
	:
	pwmPin_(pwmPin)
	, freq_(freq)
	, duty_(duty)
{
	Enable(freq,duty);
}


void PwmMyRio::Enable(int freq, double duty /* = 0 */)
{
	Pwm_Configure(&kMyRio_pwm_regs_[(int)pwmPin_],
		Pwm_Invert | Pwm_Mode,
		Pwm_NotInverted | Pwm_Enabled);
	Pwm_ClockSelect(&kMyRio_pwm_regs_[(int)pwmPin_], kClkDiv_);
	SetFreq(freq);
	SetDuty(duty);
	EditSysSelect(1);
}

void PwmMyRio::SetFreq(int freq)
{
	freq_ = freq;
	duty_max_ = kMyRioFreq_ / kClkDivides_[(int)kClkDiv_] / freq;
	Pwm_CounterMaximum(&kMyRio_pwm_regs_[(int)pwmPin_], 
		duty_max_);
}

void PwmMyRio::SetDuty(double duty)
{
	
	duty_ = duty > 1 ? 1: duty < 0 ? 0:duty;
	Pwm_CounterCompare(&kMyRio_pwm_regs_[(int)pwmPin_], duty_max_ * duty_);
}

void PwmMyRio::EditSysSelect(int bit)
{
	uint8_t selectReg;
	int status = NiFpga_ReadU8(myrio_session, kSysselct_reg[(int)pwmPin_], &selectReg);
	if (MyRio_IsNotSuccess(status))
	{
		throw std::runtime_error("error pwm enable");
	}
	if (bit == 1)
	{
		selectReg = selectReg | (1 << kSysselct_reg_bit[(int)pwmPin_]);
	}
	else
	{
		selectReg = selectReg & (~(1 << kSysselct_reg_bit[(int)pwmPin_]));
	}
	status = NiFpga_WriteU8(myrio_session, kSysselct_reg[(int)pwmPin_], selectReg);
	if (MyRio_IsNotSuccess(status))
	{
		throw std::runtime_error("error pwm enable");
	}
}
void PwmMyRio::Disable()
{
	EditSysSelect(0);
	Pwm_Configure(&kMyRio_pwm_regs_[(int)pwmPin_],
		Pwm_Invert | Pwm_Mode,
		Pwm_NotInverted | Pwm_Disabled);
}

double PwmMyRio::GetDuty()
{
	return duty_;
}
int PwmMyRio::GetFreq()
{
	return freq_;
}



PwmMyRio::~PwmMyRio()
{
	
	//Disable();
	
}
