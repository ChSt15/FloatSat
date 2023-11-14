#ifndef FLOATSAT_TELECOMMAND_HPP_
#define FLOATSAT_TELECOMMAND_HPP_

#include "stdint.h"

#include "datalink.hpp"


enum DataType_TE {
    DataType_TE_Attitude = 1, // Floatsat current attitude
    DataType_TE_Gyro = 2, // Gyro data
    DataType_TE_Mag = 3, // Magnetometer data
    DataType_TE_Acc = 4, // Accelerometer data
    DataType_TE_BatVolt = 5, // Battery voltage
    DataType_TE_BatCurr = 6, // Battery current
    DataType_TE_ReactCurr = 7, // Reaction wheel current
    DataType_TE_ReactSpeed = 8, // Reaction wheel speed
    DataType_TE_ArmPos = 9 // Arm position
};



#endif /* FLOATSAT_TELECOMMAND_HPP_ */