#ifndef FLOATSAT_DATALINK_UART_HPP_
#define FLOATSAT_DATALINK_UART_HPP_

#include "stdint.h"

#include "rodos.h"


/// @brief Abstract class for a datalink (e.g UART, WIFI, etc)
class Datalink {
public:

    virtual ~Datalink();

    /// @brief Initializes the datalink
    virtual void init() = 0;

    /// @brief Sends data over the datalink
    /// @param data Pointer to the data to be sent
    /// @param len Length of the data to be sent
    /// @return Number of bytes sent
    virtual int32_t send(uint8_t* data, uint8_t len) = 0;

    /// @brief Checks if the datalink is ready to send data
    /// @return 0 if not ready, number of bytes that can be sent otherwise
    virtual int32_t readySend() = 0;

    /// @brief Receives data over the datalink
    /// @param data Pointer to receive the received data
    /// @param len Max length of the data to be received. Extra bytes will be discarded
    /// @return Number of bytes received
    virtual int32_t receive(uint8_t* data, uint8_t len) = 0;

    /// @brief Checks if the datalink is ready to receive data
    /// @return 0 if not ready, number of bytes that can be received otherwise
    virtual int32_t readyReceive() = 0;

};





#endif /* FLOATSAT_DATALINK_HPP_ */