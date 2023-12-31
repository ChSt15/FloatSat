#ifndef FLOATSAT_THREADS_COMMUNICATIONTHREAD_HPP_
#define FLOATSAT_THREADS_COMMUNICATIONTHREAD_HPP_

#include "rodos.h"

#include "../Communication/Telemetry.hpp"
#include "../Communication/Telecomand.hpp"
#include "Config.hpp"


class CommunicationThread : public Thread
{

public:

	// Set name, prio and stack size
	CommunicationThread() : Thread("Communication Thread", 100, 2000) {}

	void init();
	void run();

private:

	int period;		// [ms]
};


extern CommunicationThread communicationthread;

#endif 
