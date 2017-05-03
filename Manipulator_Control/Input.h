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
    long velocity = 0;
    long l_steps = 0;
    long s_steps = 0;
    long l_velocity = 5000;
    long s_velocity = 5000;
    boolean resetMove = FALSE;
    bool stop_auto = false;
    char x;
    char menu;
};
class AutoInput : public Input
{
public:
    AutoInput();
    void init();
    void run(SerialChannel serial, Motors motors);
    long xsteps = 0;
    long ysteps = 0;
    long zsteps = 0;
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
private:
    bool ready_to_move = false;
    void upKey(std::string &motor, long &steps, long &velocity);
    void downKey(std::string &motor, long &steps, long &velocity);
    void rightKey(std::string &motor, long &steps, long &velocity);
    void leftKey(std::string &motor, long &steps, long &velocity);
    void checkArrowKeys(std::string &motor, long &steps, long &velocity);
    void checkOtherKeys(Motors motors, SerialChannel serial, std::string &motor);
    void commandSummary(SerialChannel serial, Motors motors);
    void motorPosition(SerialChannel serial, Motors motors);
    void sendCommandToChannel(SerialChannel serial, Motors motors);
    void resetSteps();
    void flexLiftControl(long newFlexSteps, long newLiftSteps, SerialChannel serial, Motors motors);
    void flexLiftCommand(long newFlexSteps, long newLiftSteps, SerialChannel serial, Motors motors);
    long xsteps = 0;
    long ysteps = 0;
    long zsteps = 0;
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
    long xsteps = 0;
    long ysteps = 0;
    long zsteps = 0;
private:
    bool ready_to_move = false;
//    void StartDemo(int &m, long &steps, std::string &motor, SerialChannel serial, Motors motors);
    void StartDemo(int &m, SerialChannel serial, Motors motors);
//    void resetLiftMove();
//    void resetFlexMove();
    void motorPosition(SerialChannel serial, Motors motors);
    void moveControl(SerialChannel serial, Motors motors);
    void resetSteps();
    void sendCommandToChannel(SerialChannel serial, Motors motors);
//    void raiseLeg(SerialChannel serial, Motors motors);
//    void upMove(SerialChannel serial, Motors motors);
    void flexLiftControl(long newFlexSteps, long newLiftSteps, SerialChannel serial, Motors motors);
    void flexLiftCommand(long newFlexSteps, long newLiftSteps, SerialChannel serial, Motors motors);
 //   void flexMoveRight(SerialChannel serial, Motors motors);
 //   void flexLeg(long &flexsteps, Motors &motors, long &flexPostion, SerialChannel &serial);
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
    std::string password;
    std::string manualPassword = "mm";
    std::string autoPassword = "am";
    std::string demoPassword = "legUp";
    char ch;
    const char ENTER = 13;
    int countm = 0;
};
#endif // MANUALINPUT_H
