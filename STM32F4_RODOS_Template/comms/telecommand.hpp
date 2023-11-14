#ifndef FLOATSAT_TELECOMMAND_HPP_
#define FLOATSAT_TELECOMMAND_HPP_

#include "stdint.h"

#include "datalink.hpp"


enum DataType_TC {
    DataType_TC_Shutoff = 1, // Shuts down the satellite using external switch. Has no datapacket class
    DataType_TC_Calib = 2, // Calibration command
    DataType_TC_ArmPower = 3 // Docking Arm power on/off
};


class DataPacket_TCCalib : public DataPacket {
public:

    enum Calib {
        Calib_All = 0,
        Calib_Gyro = 1,
        Calib_Mag = 2,
        Calib_Acc = 3,
        Calib_Arm = 4
    };

private:

    uint8_t calibType;

public:

    uint8_t packPacket(uint8_t* data, uint8_t len) {
        if (len < 1) return 0;
        data[0] = calibType;
        return 1;
    }

    bool unpackPacket(uint8_t* data, uint8_t len) {
        if (len < 1) return false;
        calibType = data[0];
        return true;
    }

    void setCalib(Calib calibType) {
        this->calibType = calibType;
    }

    Calib getCalib() {
        return (Calib)calibType;
    }

};



#endif /* FLOATSAT_TELECOMMAND_HPP_ */