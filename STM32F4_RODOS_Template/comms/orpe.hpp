#ifndef FLOATSAT_ORPE_HPP_
#define FLOATSAT_ORPE_HPP_

#include "stdint.h"

#include "datalink.hpp"


/// @brief Data types for ORPE
enum ORPE_DataType {
    ORPE_DataType_Power = 1, // Power ORPE on/off
    ORPE_DataType_Pose = 2 // Pose estimation from ORPE
};

class DataPacket_ORPEPose : public DataPacket {
private:

    int32_t x;
    int32_t y;
    int32_t z;
    int32_t rx;
    int32_t ry;
    int32_t rz;

    uint32_t frameNum;

public:

    uint8_t packPacket(uint8_t* data, uint8_t len) {
        if (len < 28) return 0;
        data[0] = (x >> 24) & 0xFF; data[1] = (x >> 16) & 0xFF; data[2] = (x >> 8) & 0xFF; data[3] = x & 0xFF;
        data[4] = (y >> 24) & 0xFF; data[5] = (y >> 16) & 0xFF; data[6] = (y >> 8) & 0xFF; data[7] = y & 0xFF;
        data[8] = (z >> 24) & 0xFF; data[9] = (z >> 16) & 0xFF; data[10] = (z >> 8) & 0xFF; data[11] = z & 0xFF;
        data[12] = (rx >> 24) & 0xFF; data[13] = (rx >> 16) & 0xFF; data[14] = (rx >> 8) & 0xFF; data[15] = rx & 0xFF;
        data[16] = (ry >> 24) & 0xFF; data[17] = (ry >> 16) & 0xFF; data[18] = (ry >> 8) & 0xFF; data[19] = ry & 0xFF;
        data[20] = (rz >> 24) & 0xFF; data[21] = (rz >> 16) & 0xFF; data[22] = (rz >> 8) & 0xFF; data[23] = rz & 0xFF;
        data[24] = (frameNum >> 24) & 0xFF; data[25] = (frameNum >> 16) & 0xFF; data[26] = (frameNum >> 8) & 0xFF; data[27] = frameNum & 0xFF;
        return 28;
    }

    bool unpackPacket(uint8_t* data, uint8_t len) {
        if (len < 28) return false;
        x = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
        y = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
        z = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
        rx = (data[12] << 24) | (data[13] << 16) | (data[14] << 8) | data[15];
        ry = (data[16] << 24) | (data[17] << 16) | (data[18] << 8) | data[19];
        rz = (data[20] << 24) | (data[21] << 16) | (data[22] << 8) | data[23];
        frameNum = (data[24] << 24) | (data[25] << 16) | (data[26] << 8) | data[27];
        return true;
    }

    void setPose(float x, float y, float z, float rx, float ry, float rz, uint32_t frameNum) {
        this->x = x*1000;
        this->y = y*1000;
        this->z = z*1000;
        this->rx = rx*1000;
        this->ry = ry*1000;
        this->rz = rz*1000;
        this->frameNum = frameNum;
    }

    void getPose(float& x, float& y, float& z, float& rx, float& ry, float& rz, uint32_t& frameNum) {
        x = this->x/1000.0;
        y = this->y/1000.0;
        z = this->z/1000.0;
        rx = this->rx/1000.0;
        ry = this->ry/1000.0;
        rz = this->rz/1000.0;
        frameNum = this->frameNum;
    }

    uint8_t getDataType() { Datalink::DataType::DataType_Telemetry; }
    uint8_t getDataID() { ORPE_DataType::ORPE_DataType_Pose; }

};


class DataPacket_ORPEPower : public DataPacket {
private:

    bool power;

public:

    uint8_t packPacket(uint8_t* data, uint8_t len) {
        if (len < 1) return 0;
        data[0] = power;
        return 1;
    }

    bool unpackPacket(uint8_t* data, uint8_t len) {
        if (len < 1) return false;
        power = data[0];
        return true;
    }

    void setPower(bool power) {
        this->power = power;
    }

    bool getPower() {
        return power;
    }

    uint8_t getDataType() { Datalink::DataType::DataType_Telecommand; }
    uint8_t getDataID() { ORPE_DataType::ORPE_DataType_Power; }

};



#endif /* FLOATSAT_ORPE_HPP_ */