#include "Debug_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"

static CommBuffer<TimestampedData<IMUData>> IMUDataBuffer;
static Subscriber IMUDataSubsciber(IMUDataTopic, IMUDataBuffer);

static CommBuffer<TimestampedData<Attitude_Data>> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer);


void DebugThread::init()
{

}

void DebugThread::run()
{
	TimestampedData<IMUData> IMUDataReceiver;
	TimestampedData<Attitude_Data> AttitudeDataReceiver;

	bool visual = true;
	bool calib = false;

	while (true)
	{

		IMUDataBuffer.get(IMUDataReceiver);
		AttitudeDataBuffer.get(AttitudeDataReceiver);
		if (!calib)
		{
			if (!visual)
			{
				PRINTF("Gyro: %f, %f, %f\n", IMUDataReceiver.data.angularVelocity.x, IMUDataReceiver.data.angularVelocity.y, IMUDataReceiver.data.angularVelocity.z);
				PRINTF("Accel: %f, %f, %f\n", IMUDataReceiver.data.acceleration.x, IMUDataReceiver.data.acceleration.y, IMUDataReceiver.data.acceleration.z);
				PRINTF("Mag: %f, %f, %f\n", IMUDataReceiver.data.magneticField.x, IMUDataReceiver.data.magneticField.y, IMUDataReceiver.data.magneticField.z);
				PRINTF("Temp: %f\n\n", IMUDataReceiver.data.temperature);

				YPR ypr = AttitudeDataReceiver.data.attitude.toYPR();
				PRINTF("Yaw: %f, Pitch: %f, Roll: %f\n", rad2Grad(ypr.yaw), rad2Grad(ypr.pitch), rad2Grad(ypr.roll));
				PRINTF("w: %f, %f, %f\n\n", AttitudeDataReceiver.data.angularVelocity.x, AttitudeDataReceiver.data.angularVelocity.y, AttitudeDataReceiver.data.angularVelocity.z);
			}
			else
			{
				PRINTF("%f,%f,%f,%f\n", AttitudeDataReceiver.data.attitude.q0, AttitudeDataReceiver.data.attitude.q.x, AttitudeDataReceiver.data.attitude.q.y, AttitudeDataReceiver.data.attitude.q.z);
			}
		}
		else
		{
			//PRINTF("%f, %f, %f\n", IMUDataReceiver.data.angularVelocity.x, IMUDataReceiver.data.angularVelocity.y, IMUDataReceiver.data.angularVelocity.z);
			//PRINTF("%f, %f, %f\n", IMUDataReceiver.data.acceleration.x, IMUDataReceiver.data.acceleration.y, IMUDataReceiver.data.acceleration.z);
			PRINTF("%f, %f, %f\n", IMUDataReceiver.data.magneticField.x, IMUDataReceiver.data.magneticField.y, IMUDataReceiver.data.magneticField.z);
		}

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


DebugThread debugthread;
