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
class DemoMode :public Input
{

public:
    DemoMode();
    void init();
    SerialChannel serial;
    void run(SerialChannel serial, Motors motors);
    bool stop_auto = false;
    char demo;
    char m;
    double xsteps = 0;
    double ysteps = 550;
    double zsteps = 0;

private:
    bool ready_to_move = false;
    void StartDemo(int &m, long &steps, std::string &motor, SerialChannel serial, Motors motors);
    void resetLiftMove(long &steps, std::string &motor);
    void resetFlexMove(long &steps, std::string &motor);
    void motorPosition(SerialChannel serial, Motors motors);
    void DemoMove(SerialChannel serial, Motors motors);
    void resetSteps();
    void sendCommandToChannel(SerialChannel serial, Motors motors);
    void raiseLeg(SerialChannel serial, Motors motors);
    void upMove(SerialChannel serial, Motors motors, long &steps, std::string &motor);

/*
    void downMove(long &steps, std::string &motor);
    void flexMoveDown(long &steps, std::string &motor);
    void flexMoveRight(long &steps, std::string &motor);

    void flexLeg(long &flexsteps, Motors &motors, long &flexPostion, SerialChannel &serial);
    void lowerFlexLeg(long &liftPosition, Motors &motors, long &flexPostion, long &flexsteps, long &liftsteps, SerialChannel &serial);
*/
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
    DemoMode demoMode;
    void mainKeySelection(int &x, SerialChannel serial);
};

#endif // MANUALINPUT_H
