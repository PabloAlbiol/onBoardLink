/*==============================================================================
    Pablo Albiol http://github.com/PabloAlbiol
==============================================================================*/


#ifndef LINKUDP_H
#define LINKUDP_H

#include "linkAbstract.h"

#include<cstring>
#include<arpa/inet.h>
#include<sys/socket.h>
#include <unistd.h>
#include <csignal>
#include <iostream>
#include <fcntl.h>

#define BUFFERUDP 512  // Max length of buffer


class linkUDP : public linkAbstract
{
    public:
        linkUDP(const char getipSend[], uint32_t getudpPortSend, const char getipListen[], uint32_t getudpPortListen);
        ~linkUDP();

        int8_t setupLink();
        int writeLink(const char *data, uint16_t len);
        int readLink(char *data);
        int closeLink();

    private:
        char ipSend[DEFAULT_SIZE_ARRAY];
        uint32_t udpPortSend;
        char ipListen[DEFAULT_SIZE_ARRAY];
        uint32_t udpPortListen;

        int sUDP;
        struct sockaddr_in sBind, sRecv, sSend;
        int slenSend;
        socklen_t slenRecv;

        char buffRead[BUFFERUDP];
        int16_t lenRecv;

        bool bReady;

};

#endif // LINKUDP_H
