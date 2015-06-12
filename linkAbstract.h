/*==============================================================================
    Pablo Albiol http://github.com/PabloAlbiol
==============================================================================*/


#ifndef LINKABSTRACT_H
#define LINKABSTRACT_H

#include <cstring>

#include "MAVLink/pabloag/mavlink.h"

#define DEFAULT_SIZE_ARRAY 128


class linkAbstract
{
    public:
        linkAbstract() {}
        linkAbstract(const char device[]) {}
        linkAbstract(const char getipSend[], uint32_t getudpPortSend, const char getipListen[], uint32_t getudpPortListen) {}
        virtual ~linkAbstract() {}

        virtual int8_t setupLink() = 0;
        virtual int writeLink(const char *data, uint16_t len) = 0;
        virtual int readLink(char *data) = 0;
        virtual int closeLink() = 0;

    private:

    protected:
        int32_t sizRead;
        uint8_t buffRead[MAVLINK_MAX_PACKET_LEN];
};

#endif // LINKABSTRACT_H
