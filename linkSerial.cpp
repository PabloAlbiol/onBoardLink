// Based on Robotnik. Modified by PabloAG
// Very useful information. Serial Programming Guide for POSIX Operating Systems
// http://www.cmrr.umn.edu/~strupp/serial.html

/** \file linkSerial.cpp
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2008
 *
 * \brief linkSerial Class
 * (C) 2007 Robotnik Automation, SLL
 * Class to manage a serial connection
*/


#include "linkSerial.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>


/*! \fn linkSerial::linkSerial(const char *device)
*/
linkSerial::linkSerial(const char *device, int baudrate)
{
    bReady = false;
	strcpy(cDevice, device);
	switch (baudrate)
	{
		case 0:      iBaudRate = B0; break;
		case 50:     iBaudRate = B50; break;
		case 110:    iBaudRate = B110; break;
		case 134:    iBaudRate = B134; break;
		case 150:    iBaudRate = B150; break;
		case 200:    iBaudRate = B200; break;
		case 300:    iBaudRate = B300; break;
		case 600:    iBaudRate = B600; break;
		case 1200:   iBaudRate = B1200; break;
		case 1800:   iBaudRate = B1800; break;
		case 2400:   iBaudRate = B2400; break;
		case 4800:   iBaudRate = B4800; break;
		case 9600:   iBaudRate = B9600; break;
		case 19200:  iBaudRate = B19200; break;
		case 38400:  iBaudRate = B38400; break;
		case 57600:  iBaudRate = B57600; break;
		case 115200:  iBaudRate = B115200; break;
		case 500000:  iBaudRate = B500000; break;
		default: iBaudRate = B9600; break;
	}
}

/*! \fn linkSerial::~linkSerial(void)
*/
linkSerial::~linkSerial(void)
{
    if (bReady)
    {
        closeLink();
    }
    bReady = false;
}

int8_t linkSerial::setupLink()
{
	fd = open(cDevice,  O_RDWR | O_NOCTTY | O_NDELAY); // The O_NDELAY flag tells UNIX that this program doesn't care what state the DCD signal line is in

	if(fd == -1)
	{
		return -1;
	}

	InitPort();

	fcntl(fd,F_SETFL, FNDELAY);	// The FNDELAY option causes the read function to return 0 if no characters are available on the port

	bReady = true;

	tcflush(fd,TCIOFLUSH); // Flush the input and/or output queue

	return 1;
}

// Port configuration
int linkSerial::InitPort() {

	struct termios options;
	// Get the current options for the port
	tcgetattr(fd, &options);

    // Set the baud rate
    cfsetispeed(&options, iBaudRate);
	cfsetospeed(&options, iBaudRate);

	// Enable the receiver and set local mode
	options.c_cflag |= (CLOCAL | CREAD);

	// No parity (8N1)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // Disable hardware flow control
	options.c_cflag &= ~CRTSCTS;
	// Raw input. Input characters are passed through exactly as they are received
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// Disable software flow control
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
    // Disable map CR to NL
	options.c_iflag &= ~ICRNL;

    // Raw output
	options.c_oflag &= ~OPOST;
	// Disable map NL to CR-NL
	options.c_oflag &= ~ONLCR;


	tcflush(fd, TCIFLUSH);
	// Set the new options for the port...
	tcsetattr(fd, TCSANOW, &options);

	return 1;
}

int linkSerial::available(void)
{
    int bytes_avail = 0;
    ioctl(fd,FIONREAD,&bytes_avail);
    return bytes_avail;
}

/*! \fn int linkSerial::closeLink(void)
 	* Closes serial port
*/
int linkSerial::closeLink(void)
{
    if (close(fd))
    {
        bReady = false;
        return 1;
    }
    return -1;
}

int linkSerial::readLink(char *data)
{
	if(bReady)
	{
        int n = 0;
		if ((n = read(fd, data, BUFFERSERIAL)) == -1)
		{
		    return -1;
		}
		else
		{
		    return n;
		}
	}
	else
	{
		return -1;
	}
}

int linkSerial::writeLink(const char *data, uint16_t length)
{
	if(bReady)
	{
        int n = 0;
		n = write(fd, data, length);
		return n;
	}
	else
	{
		return -1;
	}
}

/*!	\fn int linkSerial::GetBaud(void)
	*
*/
int linkSerial::GetBaud(void) {

	struct termios termAttr;
	int inputSpeed = -1;
	speed_t baudRate;
	tcgetattr(fd, &termAttr);
	// Get the input speed
	baudRate = cfgetispeed(&termAttr);
	switch (baudRate) {
		case B0:      inputSpeed = 0; break;
		case B50:     inputSpeed = 50; break;
		case B110:    inputSpeed = 110; break;
		case B134:    inputSpeed = 134; break;
		case B150:    inputSpeed = 150; break;
		case B200:    inputSpeed = 200; break;
		case B300:    inputSpeed = 300; break;
		case B600:    inputSpeed = 600; break;
		case B1200:   inputSpeed = 1200; break;
		case B1800:   inputSpeed = 1800; break;
		case B2400:   inputSpeed = 2400; break;
		case B4800:   inputSpeed = 4800; break;
		case B9600:   inputSpeed = 9600; break;
		case B19200:  inputSpeed = 19200; break;
		case B38400:  inputSpeed = 38400; break;
		case B57600:  inputSpeed = 57600; break;
		case B115200:  inputSpeed = 115200; break;
	}
	return inputSpeed;
}
