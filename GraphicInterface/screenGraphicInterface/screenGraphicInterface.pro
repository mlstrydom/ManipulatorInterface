#-------------------------------------------------
#
# Project created by QtCreator 2017-04-28T11:42:00
#
#-------------------------------------------------

QT       += core gui
QT       += gui
QT       +=widgets
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = screenGraphicInterface
TEMPLATE = app


SOURCES += main.cpp\
        screengraphics.cpp \
    interface.cpp

HEADERS  += screengraphics.h \
    interface.h

FORMS    += \
    interface.ui
