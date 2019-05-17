#include "Button.h"
extern NiFpga_Session myrio_session;
void ButtonOnMyrioIrq::Init()
{
	int32_t status;

	printf("Button Input IRQ:\n");

	/*
	* Specify the settings that correspond to the IRQ channel
	* that you need to access.
	*/
	irqBtn0_.btnIrqNumber = IRQDI_BTNNO;
	irqBtn0_.btnCount = IRQDI_BTNCNT;
	irqBtn0_.btnIrqEnable = IRQDI_BTNENA;
	irqBtn0_.btnIrqRisingEdge = IRQDI_BTNRISE;
	irqBtn0_.btnIrqFallingEdge = IRQDI_BTNFALL;

	/*
	* Open the myRIO NiFpga Session.
	* You must use this function before using all the other functions.
	* After you finish using this function, the NI myRIO target is ready to be used.
	*/


	/*
	* Configure the Button IRQ and return a status message to indicate if the configuration is successful,
	* the error code is defined in IRQConfigure.h.
	*/
	status = Irq_RegisterButtonIrq(&irqBtn0_,
		&(irqContext_),
		IrqNumberConfigure,
		CountConfigure,
		TriggerTypeConfigure);

	/*
	* Terminate the process if it is unsuccessful.

	*/
	if (status != NiMyrio_Status_Success)
	{
		throw std::runtime_error("CONFIGURE  %d, Configuration of Button IRQ failed.");
	}

	/*
	* Create new threads to catch the specified IRQ numbers.
	* Different IRQs should have different corresponding threads.
	*/
	irq_Thread_ = std::shared_ptr<std::thread>(new std::thread(&ButtonOnMyrioIrq::IrqThread,this));
	printf("Button Input IRQ: configure complite\n");
	fflush(stdout);


}


void ButtonOnMyrioIrq::IrqThread()
{
	printf("Ijyjvvyuuyfuyf");
	fflush(stdout);
	NiFpga_Bool ready = false;
	while (!stop_thread)
	{
		uint32_t irqAssert = 0;
		static uint32_t irqCount = 0;

		/*
		* Stop the calling thread, wait until a selected IRQ is asserted.
		*/
		Irq_Wait(irqContext_,
			(NiFpga_Irq)IrqNumberConfigure,
			&irqAssert,
			(NiFpga_Bool*) &(ready));
		/*
		* If an IRQ was asserted.
		*/  /*
		* If an IRQ was asserted.
		*/
		/*
		* If an IRQ was asserted.
		*/
		if (irqAssert & (1 << IrqNumberConfigure))
		{
			printf("IRQ%d,%d\n", IrqNumberConfigure, ++irqCount);
			fflush(stdout);
			++counter;
			/*
			* Acknowledge the IRQ(s) when the assertion is done.
			*/
			Irq_Acknowledge(irqAssert);
		}
	}
}


ButtonOnMyrioIrq::~ButtonOnMyrioIrq()
{
	stop_thread = true;
	irq_Thread_->join();
	Irq_UnregisterButtonIrq(&irqBtn0_,
		irqContext_,
		IrqNumberConfigure);
}


void ButtonOnMyrioIrq::WaitDown()
{
	counter_old.store(counter);
	while (!(counter > counter_old)) ;
    counter_old.store(counter);
}


ButtonOnMyrioIrq::ButtonOnMyrioIrq()
	: 
	 counter(0)
	, counter_old(0)
	, stop_thread(false)
{
	Init();
}


void ButtonOnMyrio::WaitDown()
{
	while (!IsDown())
	{
	         
	}
}


bool ButtonOnMyrio::IsDown()
{
	uint8_t val = 30;

	NiFpga_ReadU8(myrio_session, DIBTN, &val);
	return val == 1;
}


bool ButtonOnMyrioIrq::IsDown()
{
	return true;
}
