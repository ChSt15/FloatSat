#include "DockingThread.hpp"


// Subscriber for cameradata
static CommBuffer<TelemetryCamera> cameraBuffer;
static Subscriber cameraSubsciber(cameraDataTopic, cameraBuffer, "Docking Thread");

HAL_GPIO ledblue(GPIO_063);


void DockingThread::init()
{
	ledblue.init(true, 1, 0);
}

void DockingThread::run()
{
	// Wait for Electrical
	while (getMode() == Electrical_Startup) suspendCallerUntil(NOW() + 200 * MILLISECONDS);

	// Config
	using namespace config;
	{
		// Thread
		this->period = docking_thread_period;
		if (!docking_thread_enable) suspendCallerUntil(END_OF_TIME);

		// ArmController
		armController.config(max_vel, min_vel, dock_vel, max_accel, deccel_margin, steps2mm);
	}

    bool cameraState = false;
	while (true)
	{   
        cameraPwrCmdTopic.publishConst(cameraState);

		// Get new Cameradata if availible
		if (cameraBuffer.getOnlyIfNewData(cameraData.telemetryCamera))
        {	
			//cameraData.getDistance();
			//cameraData.getYawofMockup();
			//PRINTF("%f\n", rad2Grad(cameraData.getYawofMockup()));
			//PRINTF("%f\n\n", rad2Grad(cameraData.getYawtoMockup()));
			//if (cameraData.validFrame()) armController.CalcAngularVelocity(cameraData);
            //PRINTF("%f\n", rad2Grad(cameraData.getYawtoMockup()));
			//PRINTF("%f, %f\n\n", rad2Grad(cameraData.getYawtoMockup()), cameraData.getDistance());
            //Print all data from struct
            //auto &data = cameraData.telemetryCamera;
            //PRINTF("CameraData: %f %f %f \n %f %f %f \n%d %d %d %d\n", data.px, data.py, data.pz, data.rx, data.ry, data.rz, data.MeasurmentCnt, data.numLEDs, data.numPoints, data.valid);

        }

		switch (getMode())
		{
		case Idle:
            cameraState = false;
		case Standby:
            cameraState = false;
			armController.Stop();
			break;

		/* ---------------------------- Calib ---------------------------- */
		case Calib_Arm:
            cameraState = false;
			if (!armController.Calibrate()) break;

			setMode(Standby);
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:

			cameraState = true;
			armController.Calibrate();
            armController.reset();
			if (cameraData.validFrame()) setMode(Mission_Point);
			break;

		case Mission_Dock_initial:

            cameraState = true;
			if (armController.InitialExtension(cameraData)) setMode(Mission_Dock_final);		
			break;

		case Mission_Dock_final:

            cameraState = true;
			if (armController.FinalExtension(cameraData))
			{
				suspendCallerUntil(NOW() + 2 * SECONDS);

				while (!cameraData.validFrame())
				{
					ledblue.setPins(~ledblue.readPins());
					suspendCallerUntil(NOW() + period * MILLISECONDS);
					cameraBuffer.getOnlyIfNewData(cameraData.telemetryCamera);
				}
				if (abs(cameraData.getYawofMockup()) > grad2Rad(5))
				{
					armController.Calibrate();
        			armController.reset();
			 		setMode(Mission_Point);
				}
				else setMode(Mission_Pull_back);

			}
			break;

		case Mission_Pull_back:
			cameraState = true;
			if (armController.PullBack(cameraData)) setMode(Idle);
			break;

		default:
            cameraState = false;
			break;
		}

		ledblue.setPins(~ledblue.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


DockingThread dockingThread;
