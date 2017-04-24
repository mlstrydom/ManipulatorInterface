#ifndef MANUALINPUT_H
#define MANUALINPUT_H
#include <iostream>
#include <Channel.h>
#include <Motors.h>
#include <windows.h>
#include <string>
#include <conio.h>
#include <Kinematics.h>
#include <Characteristics.h>

class Input
{
public:
    virtual void run(SerialChannel serial, Motors motors) = 0; // abstract function
    long steps = 0;
    long l_steps = 0;
    long s_steps = 0;
};

class AutoInput : public Input
{
public:

    AutoInput();
    void init();
    void run(SerialChannel serial, Motors motors);
private:
    kin::Kinematics kinematics;
    Characteristics c;
};

class ManualInput : public Input
{
public:
    ManualInput();
    SerialChannel serial;
    void run(SerialChannel serial, Motors motors);
    bool stop_auto = false;
    char x;
    char menu;

private:
    bool ready_to_move = false;

    void upKey(long &steps, std::string &motor);
    void downKey(long &steps, std::string &motor);
    void rightKey(long &steps, std::string &motor);
    void leftKey(long &steps, std::string &motor);
    void checkArrowKeys(std::string &motor, long &steps);
    void checkOtherKeys(Motors motors, SerialChannel serial, long &steps, std::string &motor);
    void commandSummary();
    void sendCommandToChannel(SerialChannel serial, Motors motors, long &steps);
    void resetSteps();
};
class MainSelection
{
public:
    MainSelection();
    Motors motors;
    int x;
    void run(SerialChannel serial, Motors motors);
    ManualInput manualInput;
    AutoInput autoInput;
    void mainKeySelection(int &x, SerialChannel serial);

private:
};
#endif // MANUALINPUT_H
