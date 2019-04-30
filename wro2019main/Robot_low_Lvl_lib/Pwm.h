#pragma once
#include "MyRio_lib/PWM.h"
class Pwm
{
public:
	//@brief Enable pwm
    //@param freq frequency in Hz
	//@param duty cycle in % (100% = 1, 50% = 0.5)
	virtual void Enable(int freq, double duty = 0)= 0;
	virtual void Disable() = 0;
	//@brief Set Duty cycle pwm
	//@param duty cycle in % (100% = 1, 50% = 0.5)
	virtual void SetDuty(double duty) = 0;
	//@brief Set frequency pwm
	//@param freq frequency in Hz
	virtual void SetFreq(int freq) = 0;
	virtual double GetDuty() = 0;
	virtual int GetFreq() = 0;
};

class MyRioPwm : public Pwm
{
public:
	enum PwmPin
	{
		PWMA0 = 0,
		PWMA1 = 1,
		PWMA2 = 2,
		PWMB0 = 3,
		PWMB1 = 4,
		PWMB2 = 5,
		PWMC0 = 6,
		PWMC1 = 7
	};
	MyRioPwm(PwmPin pwmPin);
	MyRioPwm(PwmPin pwmPin, int freq, double duty = 0);
	void Enable(int freq, double duty = 0) override;
	void Disable() override;
	void SetDuty(double duty) override;
	void SetFreq(int freq) override;
	double GetDuty() override;
	int GetFreq() override;
private:
	void EditSysSelect(int bit);
	PwmPin pwmPin_;
	double freq_;
	double duty_;
	double duty_max_;
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
			SYSSELECTA, SYSSELECTA, SYSSELECTA,
			SYSSELECTB, SYSSELECTB, SYSSELECTB,
			SYSSELECTC, SYSSELECTC, 
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
	const int kMyRioFreq_ = 40000000; //Hz 
	const Pwm_ClockDivider kClkDiv_ = Pwm_8x;
	const int kClkDivides_[8] = { 0, 1, 2, 4, 8, 16, 32, 64 };
};