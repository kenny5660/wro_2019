#pragma once
#include <thread>
#include <atomic>
#include "ButtonIRQ.h"
class ButtonOnMyrio : public Button
{
public:
	ButtonOnMyrio() {}
	void  WaitDown() override;
	bool IsDown() override;	
	~ButtonOnMyrio() {}
};
class ButtonOnMyrioIrq : public Button
{
public:
	ButtonOnMyrioIrq();
	void Init();
	bool IsDown() override; 
	void WaitDown() override;
	~ButtonOnMyrioIrq();
private:
	MyRio_IrqButton irqBtn0_;
	NiFpga_IrqContext irqContext_;
	const uint8_t IrqNumberConfigure = 5;
	const uint32_t CountConfigure = 1;
	const Irq_Button_Type TriggerTypeConfigure = Irq_Button_FallingEdge;
	void IrqThread();
	std::shared_ptr<std::thread> irq_Thread_;
	std::atomic_bool stop_thread;
	std::atomic_int counter;
	std::atomic_int counter_old; 
};