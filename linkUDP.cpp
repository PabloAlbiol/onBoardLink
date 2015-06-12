/*==============================================================================
    Pablo Albiol http://github.com/PabloAlbiol
==============================================================================*/


#include "linkUDP.h"

linkUDP::linkUDP(const char getipSend[], uint32_t getudpPortSend, const char getipListen[], uint32_t getudpPortListen)
{
    bReady = false;

    strcpy(ipSend, getipSend);
    udpPortSend = getudpPortSend;
    strcpy(ipListen, getipListen);
    udpPortListen = getudpPortListen;
}

linkUDP::~linkUDP()
{
    if (bReady)
    {
        closeLink();
    }
    bReady = false;
}

int8_t linkUDP::setupLink()
{
    // Create a UDP socket
    if ((sUDP = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        return -1;
    }

    fcntl(sUDP, F_SETFL, O_NONBLOCK); // Non-blocking UDP socket

    // Zero out the structure
    memset((char *) &sBind, 0, sizeof(sBind));

    // Listen
    sBind.sin_family = AF_INET;
    sBind.sin_port = htons(udpPortListen);
    inet_aton(ipListen, (in_addr *)&sBind.sin_addr.s_addr);

    // Write
    sSend.sin_family = AF_INET;
    sSend.sin_port = htons(udpPortSend);
    inet_aton(ipSend, (in_addr *)&sSend.sin_addr.s_addr);

    // Bind socket to port
    if( bind(sUDP , (struct sockaddr*)&sBind, sizeof(sBind) ) == -1)
    {
        return -1;
    }

    slenSend = sizeof(sSend);
    bReady = true;

    return 1;
}

int linkUDP::writeLink(const char *data, uint16_t len)
{
    if (bReady)
    {
        int n;
        n = sendto(sUDP, data, len, 0, (struct sockaddr*) &sSend, slenSend);
        return n;
    }
    else
    {
        return -1;
    }
}

int linkUDP::readLink(char *data)
{
    if (bReady)
    {
        slenRecv = sizeof(sRecv);

        if ((lenRecv = recvfrom(sUDP, data, BUFFERUDP, MSG_DONTWAIT, (struct sockaddr *) &sRecv, &slenRecv)) == -1)
        {
            return -1;
        }
        else
        {
            return lenRecv;
        }
    }
    else
    {
        return -1;
    }
}

int linkUDP::closeLink()
{
    if (close(sUDP))
    {
        bReady = false;
        return 1;
    }
    return -1;
}
