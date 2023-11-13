#ifndef FLOATSAT_DATALINK_HPP_
#define FLOATSAT_DATALINK_HPP_

#include "stdint.h"


/// @brief Abstract class for a datalink physical connections (e.g UART, WIFI, etc)
class Datalink_Physical {
public:

    virtual ~Datalink_Physical();

    /// @brief Initializes the datalink physical
    virtual void initPhys() {}

    /// @brief Updates the datalink physical
    virtual void thread() {}

    /// @brief Sends data over the datalink physical
    /// @param data Pointer to the data to be sent
    /// @param len Length of the data to be sent
    /// @return Number of bytes sent
    virtual int32_t send(uint8_t* data, uint8_t len) = 0;

    /// @brief Checks if the datalink physical is ready to send data
    /// @return 0 if not ready, number of bytes that can be sent otherwise
    virtual int32_t readySend() = 0;

    /// @brief Receives data over the datalink physical
    /// @param data Pointer to receive the received data
    /// @param len Max length of the data to be received. Extra bytes will be discarded
    /// @return Number of bytes received
    virtual int32_t receive(uint8_t* data, uint8_t len) = 0;

    /// @brief Checks if the datalink physical is ready to receive data
    /// @return 0 if not ready, number of bytes that can be received otherwise
    virtual int32_t readyReceive() = 0;

};


/// @brief Abstract class for datapackets to be sent over the datalink
class DataPacket {
public:

    virtual ~DataPacket();

    /// @brief Will convert internal data to a packet for sending
    /// @param data Pointer receive the packed data
    /// @param len Max length of the pointer to receive the packed data
    /// @return 0 if failed, otherwise number of bytes packed
    virtual uint8_t packPacket(uint8_t* data, uint8_t len) const = 0;

    /// @brief Will convert a received packet to internal data
    /// @param data Pointer to the received data
    /// @param len Length of the received data
    /// @return True if the packet was successfully unpacked, false otherwise
    virtual bool unpackPacket(uint8_t const* data, uint8_t len) = 0;

    /// @brief Will return the data type e.g Telecommand, Telemetry etc
    virtual uint8_t getDataType() const = 0;

    /// @brief Will return the data ID e.g battery voltage, attitude etc
    virtual uint8_t getDataID() const = 0;

};


class Datalink final {
public:

    /// @brief Packet header
    static constexpr uint8_t HEADER = 0xAB;
    static constexpr uint8_t DEST_INDEX = 1;
    static constexpr uint8_t DATATYPE_INDEX = 2;
    static constexpr uint8_t DATAID_INDEX = 3;
    static constexpr uint8_t DATA_INDEX = 4;

    /// @brief Enum for the different types of data that can be sent over the datalink
    enum DataType {
        DataType_Telecommand = 0,
        DataType_Telemetry = 1,
        DataType_CameraStream = 2
    };

    /// @brief Enum for the different destinations that can be sent over the datalink
    enum Destination {
        Destination_GS = 0, // Ground Station
        Destination_ORPE = 1, // ORPE (Optical Relative Pose Estimation)
        Destination_FS = 2 // FloatSat
    };

private:

    uint8_t sendBuffer[255];
    uint8_t sendBufferLen;

    uint8_t receiveBuffer[255];
    uint8_t receiveBufferLen;

    uint8_t receiveCounter;

    Datalink_Physical& physical;

    Destination nodeType;

    uint8_t latestDataType;
    uint8_t latestDataID;

public:

    /// @brief Constructor
    /// @param physical Physical datalink to be used
    /// @param nodeType What type of node this datalink is
    Datalink(Datalink_Physical& physical, Destination nodeType);

    /// @brief Will check for new data from the physical datalink or send data to it
    void update();

    /// @brief Send the given packet over the datalink to the given destination
    /// @param dest Destination to send the packet to
    /// @param packet Packet to send
    /// @return True if the packet was successfully sent, false otherwise
    bool send(uint8_t dest, const DataPacket& packet);

    /// @return the type of data that was received. e.g. Telecommand, Telemetry etc
    uint8_t getDataType() const;
    /// @return the ID of the data that was received. e.g battery voltage, attitude etc
    uint8_t getDataID() const;

    /// @brief Will return a counter of the number of packets received. Used to identify if a packet is new.
    uint8_t getReceiveCounter() const;
    
    /// @brief Will place recieved data into the packet
    /// @param packet Packet to place the received data into
    /// @return True if the packet was successfully received, false otherwise
    bool receive(DataPacket& packet);

};





#endif /* FLOATSAT_DATALINK_HPP_ */