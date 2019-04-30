#include "Pwm.h"
#include "MyRio_lib/PWM.h"
#include <memory>
extern NiFpga_Session myrio_session;


MyRioPwm::MyRioPwm(PwmPin pwmPin)
	: pwmPin_(pwmPin)
{
}

MyRioPwm::MyRioPwm(PwmPin pwmPin, 
	int freq, 
	double duty /* = 0 */)
	:
	pwmPin_(pwmPin)
	, freq_(freq)
	, duty_(duty)
{
	Enable(freq,duty);
}


void MyRioPwm::Enable(int freq, double duty /* = 0 */)
{
	Pwm_Configure(&kMyRio_pwm_regs_[(int)pwmPin_],
		Pwm_Invert | Pwm_Mode,
		Pwm_NotInverted | Pwm_Enabled);
	Pwm_ClockSelect(&kMyRio_pwm_regs_[(int)pwmPin_], kClkDiv_);
	SetFreq(freq);
	SetDuty(duty);
	EditSysSelect(1);
}

void MyRioPwm::SetFreq(int freq)
{
	freq_ = freq;
	duty_max_ = kMyRioFreq_ / kClkDivides_[(int)kClkDiv_] / freq;
	Pwm_CounterMaximum(&kMyRio_pwm_regs_[(int)pwmPin_], 
		duty_max_);
}

void MyRioPwm::SetDuty(double duty)
{
	
	duty_ = duty > 1 ? 1: duty < 0 ? 0:duty;
	Pwm_CounterCompare(&kMyRio_pwm_regs_[(int)pwmPin_], duty_max_ * duty_);
}

void MyRioPwm::EditSysSelect(int bit)
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
void MyRioPwm::Disable()
{
	EditSysSelect(0);
	Pwm_Configure(&kMyRio_pwm_regs_[(int)pwmPin_],
		Pwm_Invert | Pwm_Mode,
		Pwm_NotInverted | Pwm_Disabled);
}

double MyRioPwm::GetDuty()
{
	return duty_;
}
int MyRioPwm::GetFreq()
{
	return freq_;
}

