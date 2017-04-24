#include "Channel.h"
#include <QString>
#include <QByteArray>
#include <QtCore/QtGlobal>
#include <QList>
#include <QFileInfo>
#include <QFileInfoList>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <chrono>
#include <windows.h>


Channel::Channel()
{
}
void SerialChannel::connect(std::string com_port)
{
    QString s = QString::fromStdString(com_port);
    port = new QextSerialPort(s);

    port->setBaudRate(BAUD115200);
    port->setFlowControl(FLOW_OFF);
    port->setParity(PAR_NONE);
    port->setDataBits(DATA_8);
    port->setStopBits(STOP_1);

    port->open(QIODevice::ReadWrite);

    if(port->isOpen())
    {
        std::cout << "Setting up Comm Port - Please wait... " << std::endl;
        Sleep(3000);
        std::cout << "Done - listening for data on: " << com_port << std::endl;
        std::cout << "Usage: " << std::endl
                  << "\t Use 'd' key for DEMO Mode" << std::endl
                  << "\t Use 'm' key for Manual mode" << std::endl
                  << "\t Use 'a' key for AUTO Control Mode" << std::endl
                  << std::endl;
    }
    else
    {
        std::cout << "Port failed to open on: " << com_port ;
        abort();
    }

}

void SerialChannel::disconnect(std::string com_port)
{
    port->close();
}

std::string SerialChannel::rx()
{
    QByteArray data;

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    while(data.isEmpty() ||
          data.at(0) != '#' ||
          data.at(data.length()-1) != '$'){
        data.append(port->readAll());

        //Add a timeout to stop an infinate loop.
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > 1000){
            std::cout << "rx timeout" << std::endl;
            break;
        }
    }

    //print using string
    std::string stdStringData(data.constData(), data.length());
    port->flush();
    return stdStringData;
}

void SerialChannel::tx(std::string data)
{
    port->write(data.c_str());
}
