#ifndef HAND_BRIDGE_HARDWARE_H_
#define HAND_BRIDGE_HARDWARE_H_

#include <boost/chrono.hpp>
#include <string>

#include "pigpio.h"

#include <boost/lexical_cast.hpp>

class HandBridgeHardware {
    boost::chrono::steady_clock::time_point start;
    int ser_port;

    bool isOpen() const{
        return ser_port >= 0 && serDataAvailable(ser_port) >= 0;
    }
public:
    HandBridgeHardware(): ser_port(-1) {}
    void init(char * param)
    {
        unsigned int  baud = 57600;
        if(char * delim = strchr(param, '@')){
            *delim = 0;
            baud = boost::lexical_cast<unsigned int>(delim + 1);
        }
        ser_port = serOpen(param, baud, 0);
        if(!isOpen()) exit(1);
    }
    unsigned long time()
    {
        return boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::steady_clock::now() - start).count();
    }
    int read(){
        return serReadByte(ser_port);
    }
    void write(uint8_t* data, int length)
    {
        serWrite(ser_port, (char *) data, length);
    }
};

#endif // HAND_BRIDGE_HARDWARE_H_