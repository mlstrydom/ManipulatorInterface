#include "Input.h"

ManualInput::ManualInput()
{

}

void ManualInput::upKey(long &steps, std::string &motor)
{
    std::cout << "up" << std::endl;
    motor = "l";
    steps = 100;
    l_steps += steps;
    std::cout << "motor = " << motor <<
                 " steps = " << l_steps <<
                 std::endl;
}
void ManualInput::downKey(long &steps, std::string &motor)
{
    std::cout << "down" << std::endl;
    motor = "l";
    steps = -100;
    l_steps += steps;
    std::cout << "motor = " << motor <<
                 " steps = " << l_steps <<
                 std::endl;
}
void ManualInput::rightKey(long &steps, std::string &motor)
{
    std::cout << "right" << std::endl;
    motor = "f";
    steps = 500;
    s_steps += steps;
    std::cout << "motor = " << motor <<
                 " steps = " << s_steps <<
                 std::endl;
}
void ManualInput::leftKey(long &steps, std::string &motor)
{
    std::cout << "left" << std::endl;
    motor = "f";
    steps = -500;
    s_steps += steps;
    std::cout << "motor = " << motor <<
                 " steps = " << s_steps <<
                 std::endl;
}
void ManualInput::checkArrowKeys(std::string &motor, long &steps)
{
    x=getch();

    switch(x) {
        case 72:    // key up
            upKey(steps, motor);
            break;
        case 80:    // key down
            downKey(steps, motor);
            break;
        case 77:    // key right
            rightKey(steps, motor);
            break;
        case 75:    // key left
            leftKey(steps, motor);
            break;
        default:
            printf("Unknown keyboard input: ascii = %d\n",(int) x);
            break;
    }
}

void ManualInput::commandSummary()
{
    std::cout << "Summary of commands:" << std::endl;
    std::cout << "s_motor = " <<
                 s_steps <<
                 " steps" <<
                 std::endl;
    std::cout << "l_motor = " <<
                 l_steps <<
                 " steps" <<
                 std::endl;
    ready_to_move = true;
}

void ManualInput::resetSteps()
{
    l_steps = 0;
    s_steps = 0;
}

void ManualInput::sendCommandToChannel(SerialChannel serial, Motors motors, long &steps)
{

    if(ready_to_move == true){
        serial.tx(motors.commandFactory("f", s_steps));
        std::cout << "Response...";
        std::string stdStringData = serial.rx();
        std::cout << "data = " << stdStringData << std::endl;

        serial.tx(motors.commandFactory("l", l_steps));
        std::cout << "Response...";
        std::string stdStringData1 = serial.rx();
        std::cout << "data = " << stdStringData1 << std::endl;

        resetSteps();
        std::cout << "Motor commands sent" << std::endl;
    }else{
        std::cout << "You must first review the commands with by typing 'r'" << std::endl;
    }
    ready_to_move = false;
}


void ManualInput::checkOtherKeys(Motors motors, SerialChannel serial, long &steps, std::string &motor)
{
    switch(x) {
        case 113: // q
            std::cout << "Exiting Program" << std::endl;
            exit(0);
        case 114: // r
            commandSummary();
            break;
        case 97: // a
            resetSteps();
            stop_auto = false;
            break;
        case 13: // enter
            sendCommandToChannel(serial, motors, steps);
            break;
        case 105: // i
            std::cout << "Manual input. Usage: motor steps" << std::endl;
            char line[100];
            char sscanf_motor[100];
            long manual_steps;
            std::cin.getline( line, 100, '\n' );
            sscanf(line, "%s %ld", &sscanf_motor, &manual_steps);
            motor = (std::string)sscanf_motor;
            if(motor.compare("f") == 0){
                s_steps = manual_steps;
            }else if(motor.compare("l") == 0){
                l_steps = manual_steps;
            }else{
                std::cout << "Unknown motor type" << std::endl;
            }
            std::cout << "motor = " << motor <<
                         " steps = " << manual_steps <<
                         std::endl;
            break;
        default:
            printf("Unknown keyboard input in manual mode: ascii = %d\n",(int) x);
            break;
    }
}

void ManualInput::run(SerialChannel serial, Motors motors)
{
    if(kbhit()){
        stop_auto = true;
        std::string motor;
        long steps;
        x = getch();
        if(x==0 || x==-32)
        {
            ready_to_move = false;
            checkArrowKeys(motor, steps);
        }else{
            checkOtherKeys(motors, serial, steps, motor);
        }
    }
}

//---------------AutoInput-----------------------------
AutoInput::AutoInput()
{
    init();
}

void AutoInput::init()
{    //Define s motor
    c.s.screwPitch = 20;
    c.s.screwLength = 600;
    c.s.motorDegPerStep = 1.8;
    c.s.motorMicroStep = 800;
    //Define l motor
    c.l.screwPitch = 20;
    c.l.screwLength = 300;
    c.l.motorDegPerStep = 1.8;
    c.l.motorMicroStep = 800;

}

void AutoInput::run(SerialChannel serial, Motors motors)
{
    serial.tx(motors.commandFactory("p", 0));
    std::string data = serial.rx();
    //Find the unique identifier cpos
    std::size_t found = data.find("cpos");
    long x_steps = 0, y_steps = 0 ,z_steps = 0;
    double x = 0, y = 0, z = 0;
    char a;
    char cposIdentify[10];
    if (found!=std::string::npos){ //no 'cpos' substring in data
        sscanf(data.c_str(),"%c %s %ld %ld %c",&a, &cposIdentify, &z_steps, &y_steps,&a);
        std::cout <<  data << std::endl;
        std::cout << "z_steps = " << z_steps << std::endl;
        std::cout << "y_steps = " << y_steps << std::endl;
    }
    Characteristics c;
    c.s.screwPitch = 20;
    c.s.motorDegPerStep = 1.8;
    c.s.motorMicroStep = 4;

    c.l.screwPitch = 20;
    c.l.motorDegPerStep = 1.8;
    c.l.motorMicroStep = 6;

    //TODO: fix rotation in dh so x=x, y=y, z=z. currently z=x, y=y, z=x.
    z = c.stepsToMillimeters(x_steps,
                         c.l.screwPitch,
                         c.l.motorDegPerStep,
                         c.l.motorMicroStep);

    y = c.stepsToMillimeters(y_steps,
                         c.l.screwPitch,
                         c.l.motorDegPerStep,
                         c.l.motorMicroStep);

    x = c.stepsToMillimeters(z_steps,
                         c.s.screwPitch,
                         c.s.motorDegPerStep,
                         c.s.motorMicroStep);

    std::cout << "x from arduino = " << x << std::endl;
    std::cout << "y from arduino = " << y << std::endl;
    std::cout << "z from arduino = " << z << std::endl;


    kinematics.ikin.updateCurrentState(x, y, z);
    static kin::state s = {0,0,0,0,0,0};
    std::cout << "sx = " << s.x << std::endl;
    std::cout << "x = " << x << std::endl;
    std::cout << "ex = " << (std::pow(s.x,2)-std::pow(x,2)) << std::endl;
    std::cout << "sy = " << s.x << std::endl;
    std::cout << "y = " << x << std::endl;
    std::cout << "ey = " << (std::pow(s.x,2)-std::pow(x,2)) << std::endl;
    std::cout << "sz = " << s.x << std::endl;
    std::cout << "z = " << x << std::endl;
    std::cout << "ez = " << (std::pow(s.x,2)-std::pow(x,2)) << std::endl;
    if(
            (std::pow(s.x,2)-std::pow(x,2))<=1e-5
            && (std::pow(s.y,2)-std::pow(y,2))<=1e-5
            && (std::pow(s.z,2)-std::pow(z,2))<=1e-5
       ){
    s = kinematics.process();
    long s_auto_steps = c.millimetersToSteps(s.x,
                         c.s.screwPitch,
                         c.s.motorDegPerStep,
                         c.s.motorMicroStep);

    long l_auto_steps = c.millimetersToSteps(s.y,
                         c.l.screwPitch,
                         c.l.motorDegPerStep,
                         c.l.motorMicroStep);

    std::cout << "--->s to arduino = " << s_auto_steps << std::endl;
    std::cout << "--->l to arduino = " << l_auto_steps << std::endl;

    serial.tx(motors.commandFactory("f", s_auto_steps));
    std::cout << "Response...";
    std::string stdStringData = serial.rx();
//    std::cout << "auto data = " << stdStringData << std::endl;

    serial.tx(motors.commandFactory("l", l_auto_steps));
    std::cout << "Response...";
    std::string stdStringData1 = serial.rx();
//    std::cout << "auto data = " << stdStringData1 << std::endl;
    }
}
//---------------Main Selection-----------------------------
MainSelection::MainSelection()
{

}
void MainSelection::mainKeySelection(int &x, SerialChannel serial)
{
    switch(x) {
        case 100: // d
            std::cout << "DEMO Mode" << std::endl;
            exit(0);
        case 109: // m
            std::cout << "Manual Input Mode" << std::endl;
            std::cout << "Usage: " << std::endl
                      << "\t Use Left and Right arrow key for slider motor" << std::endl
                      << "\t Use Up and Down arrow key for Lift motor" << std::endl
                      << "\t Use 'i' key for manual step entry (motor (f or l) steps)" << std::endl
                      << "\t Use 'r' key for step summary" << std::endl
                      << "\t Use 'Enter' key to send steps to motors" << std::endl
                      << std::endl;
                while(true){
                    manualInput.run(serial,motors);
                    if(!manualInput.stop_auto){
     //                   autoInput.run(serial,motors);
                }
                    //Sleep(1000);
               }
                break;
        case 97: // a
            std::cout << "AUTO Mode" << std::endl;
//            autoInput.run(serial, motors);
            break;
        default:
            printf("Unknown keyboard input in main menu: ascii = %d\n",(int) x);
            break;
    }
    }

