#include "Control_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

#include "../hardware/hbridge.hpp"
#include "../hardware/adc.hpp"


HAL_GPIO ledorange(GPIO_061);

void ControlThread::init()
{
	/**
	 * Initialize HBridge
	*/
	hbridge.initialization();
	
	/**
	 * Initialize ADC
	*/
	adc.initialization();

	ledorange.init(true, 1, 0);
}



void ControlThread::run()
{
	// Config
	using namespace config;
	{
		// Thread
		this->period = control_thread_period;
		if (!control_thread_enable) suspendCallerUntil(END_OF_TIME);
	}

	float desiredVoltage;

	while(true)
	{
		desiredVoltage = adc.getVoltage();
		hbridge.setVoltage(desiredVoltage);

		ledorange.setPins(~ledorange.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


ControlThread controlthread;
