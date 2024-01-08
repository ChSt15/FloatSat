#ifndef FLOATSAT_THREADS_CONTROLTHREAD_HPP_
#define FLOATSAT_THREADS_CONTROLTHREAD_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"
#include "../hardware/hbridge.hpp"
#include "../hardware/adc.hpp"
#include "Config.hpp"


class ControlThread : public Thread
{
public:

	// Set name, prio and stack size
	ControlThread() : Thread("Control Thread", 100, 2000) {}

	void init();
	void run();

private:

	int period;		// [ms]
};


extern ControlThread controlthread;

#endif 
