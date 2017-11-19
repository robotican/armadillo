#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <Arduino.h>
class Communicator
{


public:
	void init(int baudrate);
	void send(byte buff[], size_t size);
	int read(byte buff[], size_t size);
};

#endif //COMMUNICATOR_H
