#ifndef FLOATSAT_HARDWARE_REACTIONWHEELENCODER_HPP_
#define FLOATSAT_HARDWARE_REACTIONWHEELENCODER_HPP_

#include "rodos.h"
#include "../timestamp.hpp"

/**
 * @notes!!!!!!!!!!!!!!!!!1 
 * - Check the class consts. They are only temporary values!
*/

extern Topic<TimestampedData<float>> EncoderDataTopic;

class ReactionwheelEncoder
{

public:

	ReactionwheelEncoder();

	void Init();

	// @brief Gets speed of reactionwheel
	// @return reactionwheel speed RAD/s
	TimestampedData<float>& getSpeed();

private:

	TimestampedData<float> Speed;
};


extern ReactionwheelEncoder encoder;

#endif
