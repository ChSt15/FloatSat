#ifndef FLOATSAT_COMMUNICATION_TELECOMAND_HPP_
#define FLOATSAT_COMMUNICATION_TELECOMAND_HPP_

#include "rodos.h"

#include "TelecommandList.hpp"
#include "Telemetry.hpp"


class Telecommand
{
private:

	int commandCnt = 0;

public:

	// @brief Processes the last new command received (if there is one)
	void processNewCommand();

	// @brief Returns number of successfully received commands
	int getCommandCounter();
};

// @brief Global telecommand object
extern Telecommand telecommand;

// @brief Global telecommand topic
extern Topic<Command> telecommandTopic;

extern Topic<float> telecommandtestTopic;

#endif