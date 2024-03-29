#ifndef FLOATSAT_DOCKING_DOCKINGTHREAD_HPP_
#define FLOATSAT_DOCKING_DOCKINGTHREAD_HPP_

#include "rodos.h"

#include "ArmControl.hpp"
#include "StepperMotorThread.hpp"
#include "../Communication/Camera.hpp"

#include "../Modes.hpp"
#include "../Config.hpp"

class DockingThread : public Thread
{

public:

	// Set name, prio and stack size
	DockingThread() : Thread("Docking Thread", 140, 2000) {}

	void init();
	void run();

private:

	int period = 200;		// [ms]

	CameraData cameraData;
};


extern DockingThread dockingThread;

#endif 
