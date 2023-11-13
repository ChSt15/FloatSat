#include "stdint.h"

#include "../datalink.hpp"


Datalink::Datalink(Datalink_Physical& physical, Destination nodeType) :
    physical(physical),
    nodeType(nodeType)
{}

void Datalink::update() {
    
    if (sendBufferLen > 0 && physical.readySend() >= sendBufferLen) {
        physical.send(sendBuffer, sendBufferLen);
        sendBufferLen = 0;
    }

    if (receiveBufferLen == 0 && physical.readyReceive() > 0) {
        uint8_t len = physical.receive(receiveBuffer, 255);
        if (len > 0) {
            receiveBufferLen = len;
            receiveCounter++;
            latestDataType = receiveBuffer[DATATYPE_INDEX];
            latestDataID = receiveBuffer[DATAID_INDEX];
        }
    }

}

bool Datalink::send(uint8_t dest, const DataPacket& packet) {
    sendBuffer[0] = 0xAB;
    sendBuffer[DEST_INDEX] = dest;
    sendBuffer[DATATYPE_INDEX] = packet.getDataType();
    sendBuffer[DATAID_INDEX] = packet.getDataID();
    uint8_t ret = packet.packPacket(sendBuffer + DATA_INDEX, 255 - DATA_INDEX);
    if (!ret) return false;
    sendBufferLen = ret + 4;
    return true;
}

uint8_t Datalink::getDataType() const {
    return latestDataType;
}

uint8_t Datalink::getDataID() const {
    return latestDataID;
}

uint8_t Datalink::getReceiveCounter() const {
    return receiveCounter;
}
    
bool Datalink::receive(DataPacket& packet) {
    if (receiveBufferLen == 0) return false;
    if (receiveBuffer[0] != 0xAB) return false;
    if (receiveBuffer[DEST_INDEX] != nodeType) return false;
    uint8_t dataType = receiveBuffer[DATATYPE_INDEX];
    uint8_t dataID = receiveBuffer[DATAID_INDEX];
    bool ret = packet.unpackPacket(receiveBuffer + DATA_INDEX, receiveBufferLen - DATA_INDEX);
    return ret;
}
