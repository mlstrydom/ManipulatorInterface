#ifndef CHANNEL_H
#define CHANNEL_H
#include<string>
#include <iostream>
#include <fstream>

#include <3rdparty/qextserialport/src/qextserialport.h>
#include <3rdparty/qextserialport/src/qextserialenumerator.h>




class Channel
{
public:
    Channel();
    virtual void connect(std::string com_port) = 0;
    virtual void disconnect(std::string com_port) = 0;
    virtual std::string rx() = 0;
    virtual void tx(std::string data) = 0;
};

class SerialChannel : public Channel
{
public:
    void connect(std::string com_port);
    void disconnect(std::string com_port);
    std::string rx();
    void tx(std::string data);
private:
    QextSerialPort *port;
};

#endif // CHANNEL_H
