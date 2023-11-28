#ifndef FLOATSAT_THREADS_DEBUGTHREAD_HPP_
#define FLOATSAT_THREADS_DEBUGTHREAD_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../timestamp.hpp"


/**
 * @brief Use this class/thread for testing different functions.
*/
class TestingThread : public Thread
{
public:

	// Set name, prio and stack size
	TestingThread();// : Thread("Debug Thread", 100, 2000) {} 

	void init();
	void run();

private:

	const int period = 10;		// [ms]
};


//extern TestingThread debugthread;

#endif 