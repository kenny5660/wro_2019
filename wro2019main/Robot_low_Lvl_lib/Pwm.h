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
	virtual ~Pwm() {};
};

class PwmMyRio : public Pwm
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
	PwmMyRio(PwmPin pwmPin);
	//@brief Create and Enable pwm
    //@param freq frequency in Hz
	//@param duty cycle in % (100% = 1, 50% = 0.5)
	PwmMyRio(PwmPin pwmPin, int freq, double duty = 0);
	void Enable(int freq, double duty = 0) override;
	void Disable() override;
	void SetDuty(double duty) override;
	void SetFreq(int freq) override;
	double GetDuty() override;
	int GetFreq() override;
	 virtual ~PwmMyRio();
private:
	void EditSysSelect(int bit);
	PwmPin pwmPin_;
	double freq_;
	double duty_;
	double duty_max_;
};