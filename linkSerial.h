// Based on Robotnik, Modified by PabloAG
/** \file linkSerial.h
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2008
 *
 * \brief linkSerial Class
 * (C) 2007 Robotnik Automation, SLL
 * Class to manage a serial port connection
*/

#ifndef LINKSERIAL_H
#define LINKSERIAL_H

#include "linkAbstract.h"

#define BUFFERSERIAL 254


class linkSerial : public linkAbstract {

public:

	linkSerial(const char *device, int baudrate);
	~linkSerial(void);

	int available(void);
	int8_t setupLink();
	int closeLink();
	int readLink(char *data);
	int writeLink(const char *data, uint16_t length);

private:

	int fd;
    bool bReady;

	// Device's name
	char cDevice[DEFAULT_SIZE_ARRAY];

	// BaudRate: 9600, 19200, 38400, 115200...
	int iBaudRate;

private:

	int InitPort();
	int GetBaud(void);
};

#endif // LINKSERIAL_H
