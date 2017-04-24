QT += core gui serialport

include(3rdparty/qextserialport/src/qextserialport.pri)

TEMPLATE = app
CONFIG += console

#CONFIG += link_pkgconfig
#PKGCONFIG += opencv

INCLUDEPATH += C:\\opencv\\build\\include

LIBS += -LC:\\opencv\\build\\x64\\vc14\\lib \
    -lopencv_world310d \

SOURCES += main.cpp \
    Kinematics.cpp \
    Channel.cpp \
    Characteristics.cpp \
    Segmentation.cpp \
    Motors.cpp \
    Input.cpp

HEADERS += \
    Kinematics.h \
    Channel.h \
    Characteristics.h \
    Segmentation.h \
    Motors.h \
    Input.h

