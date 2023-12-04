#include "Telecomand.hpp"

// Dummy topic + sub
Topic<char> DummyTopic(42, "Dummy");
static CommBuffer<char> DummyBuffer;
static Subscriber DummyDataSubsciber(DummyTopic, DummyBuffer);
char DummyReceiver = 'c';


Telecomand::Telecomand()
{
	// topics to forward
	topics.add(DummyTopic.topicId);
	//..
	uart_gateway.setTopicsToForward(&(this->topics));
}


void Telecomand::process()
{
	//DummyBuffer.get(DummyReceiver);
	DummyTopic.publish(DummyReceiver);
	//telemetry.temp_test(DummyReceiver);

}



Telecomand telecomand;
