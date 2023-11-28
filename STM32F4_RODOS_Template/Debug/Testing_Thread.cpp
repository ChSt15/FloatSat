#include "Testing_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"
#include "../hardware/hbridge.hpp"

#include "simple_pid.hpp"

TestingThread::TestingThread() : Thread("Testing Thread", 100, 2000)
{

}

void TestingThread::init()
{

	hbridge.initialization();
	encoder.Init();

}

void TestingThread::run()
{

	VCTR::CTRL::SimplePID pid(0.1, 0, 0, 0.4);

	while (true)
	{

		float wheelVel = encoder.getSpeed().data;

		float wheelPwr = pid.update(wheelVel);
		wheelPwr = sinf(SECONDS_NOW() * 0.1) * 0.5f;

		hbridge.setVoltage(wheelPwr);

		PRINTF("Power: %.2f [%], Encoder: %.2f [Rad/s]", wheelPwr, wheelVel);

		suspendCallerUntil(NOW() + period * MILLISECONDS);
		
	}

}

TestingThread testingThread;

//DebugThread debugthread;
//Semaphore mode_protec;
//modes mode = Idle;
