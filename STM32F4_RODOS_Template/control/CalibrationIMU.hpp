#ifndef FLOATSAT_CONTROL_CALIBRATIONIMU_HPP_
#define FLOATSAT_CONTROL_CALIBRATIONIMU_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"
#include "../hardware/imu.hpp"


class CalibrationIMU
{
private:

    int gyro_maxsamples = 200;
    int accel_maxsamples = 200;
    int mag_maxsamples = 500;

    int gyro_samples = 0;
    int accel_samples = 0;
    int mag_samples = 0;

    Vector3D_F gyro_sum = Vector3D_F(0, 0, 0);
    Vector3D_F accel_sum = Vector3D_F(0, 0, 0);

    float mag_minx, mag_miny =  42000.0f;
    float mag_maxx, mag_maxy = -42000.0f;

public:

    bool calibrateGyro(TimestampedData<IMUData> imurawdata);

    bool calibrateAccel(TimestampedData<IMUData> imurawdata);

    bool calibrateMag(TimestampedData<IMUData> imurawdata);

};


extern CalibrationIMU imucalib;

#endif
