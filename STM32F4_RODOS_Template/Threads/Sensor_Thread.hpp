#ifndef FLOATSAT_THREADS_SENSORTHREAD_HPP_
#define FLOATSAT_THREADS_SENSORTHREAD_HPP_

#include "rodos.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../timestamp.hpp"
#include "Modes.hpp"
#include "Config.hpp"


class SensorThread : public Thread
{

public:

	// Set name, prio and stack size
	SensorThread() : Thread("Sensor Thread", 100, 20000) {}

	void init();
	void run();

private:

	int period;		// [ms]
};


extern SensorThread sensorthread;

#endif 
