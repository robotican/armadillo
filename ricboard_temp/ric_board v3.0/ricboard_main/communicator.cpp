#include "communicator.h"

void Communicator::init(int baudrate)
{
	Serial.begin(baudrate);
}

void Communicator::send(byte buff[], size_t size)
{
	Serial.write(buff, size);
}

/* return: number of bytes read */
int Communicator::read(byte buff[], size_t size)
{
  int indx=0;
	for (; indx<size && Serial.available()>0; indx++)
	{
		buff[indx] = Serial.read();
	}
	return indx;
}
