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
{    //Define f motor
    c.s.screwPitch = 20;
    c.s.screwLength = 600;
    c.s.motorDegPerStep = 1.8;
    c.s.motorMicroStep = 4;
    //Define l motor
    c.l.screwPitch = 20;
    c.l.screwLength = 300;
    c.l.motorDegPerStep = 1.8;
    c.l.motorMicroStep = 6;

}

void AutoInput::run(SerialChannel serial, Motors motors)
{
    serial.tx(motors.commandFactory("p", 0));
    std::string data = serial.rx();
    //Find the unique identifier cpos
    std::size_t found = data.find("cpos");
    long x_steps = 0, y_steps = 0 ,z_steps = 0;
    double xmm = 0, ymm = 0, zmm = 0;
    char a;
    char cposIdentify[100];
    if (found!=std::string::npos){ //no 'cpos' substring in data
        sscanf(data.c_str(),"%c %s %ld %ld %c",&a, &cposIdentify, &z_steps, &y_steps,&a);
        std::cout << "data from Arduino =:" <<data << std::endl;
        std::cout << "Arduino z_steps = " << z_steps << std::endl;
        std::cout << "Arduino y_steps = " << y_steps << std::endl;
        std::cout << "---" << std::endl;

    }
    //TODO: fix rotation in dh so x=x, y=y, z=z. currently z=x, y=y, z=x.
    zmm = c.stepsToMillimeters(x_steps,  //Used z for flex (not x)
                         c.l.screwPitch,
                         c.l.motorDegPerStep,
                         c.l.motorMicroStep);

    ymm = c.stepsToMillimeters(y_steps,
                         c.l.screwPitch,
                         c.l.motorDegPerStep,
                         c.l.motorMicroStep);

    xmm = c.stepsToMillimeters(z_steps,
                         c.s.screwPitch,
                         c.s.motorDegPerStep,
                         c.s.motorMicroStep);

    std::cout << "x distance in mm moved by motor = " << xmm << std::endl;
    std::cout << "y distance in mm moved by motor = " << ymm << std::endl;
    std::cout << "z distance in mm moved by motor = " << zmm << std::endl;
    std::cout << "---------------------------------------------" << std::endl;


    kinematics.ikin.updateCurrentState(xmm, ymm, zmm);
    static kin::state s = {0,0,0,0,0,0};
    std::cout << "sx from matrix = " << s.x << std::endl;
    std::cout << "x distance moved by motor = " << xmm << std::endl;
    std::cout << "x squared step error (ex) = " << (std::pow(s.x,2)-std::pow(xmm,2)) << std::endl;
    std::cout << "sy = from matrix " << s.y << std::endl;
    std::cout << "y = distance moved by motor " << ymm << std::endl;
    std::cout << "y squared step error (ey)" << (std::pow(s.y,2)-std::pow(ymm,2)) << std::endl;
    std::cout << "sz = from matrix " << s.z << std::endl;
    std::cout << "z = distance moved by motor " << zmm << std::endl;
    std::cout << "z squared step error (ez)" << (std::pow(s.z,2)-std::pow(zmm,2)) << std::endl;
    std::cout << "---" << std::endl;

    if(  //check how if given and reached position is the same
            (std::pow(s.x,2)-std::pow(xmm,2))<=1e-5
            && (std::pow(s.y,2)-std::pow(ymm,2))<=1e-5
            && (std::pow(s.z,2)-std::pow(zmm,2))<=1e-5
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

    std::cout << "--->Flex steps to arduino = " << s_auto_steps << std::endl;
    std::cout << "--->Lift steps to arduino = " << l_auto_steps << std::endl;

    serial.tx(motors.commandFactory("f", s_auto_steps));
    std::cout << "Arduino Response...";
    std::string stdStringData = serial.rx();
    std::cout << "f auto data = " << stdStringData << std::endl;

    serial.tx(motors.commandFactory("l", l_auto_steps));
    std::cout << "Arduino Response...";
    std::string stdStringData1 = serial.rx();
    std::cout << "L auto data = " << stdStringData1 << std::endl;
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
            std::cout << "MANIPULATOR DEMO MODE" << std::endl;
            std::cout << "\tUsage: " << std::endl
                      << "\t (i)  Use 'q' to quit demo program" << std::endl
                      << "\t (ii) Press ENTER to continue with the demonstation" << std::endl
                      << std::endl;
            while(true){
                    demoMode.run(serial,motors);
                    Sleep(1000);
            }
             break;
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
                    autoInput.run(serial,motors);
            }
           }
        break;
        case 97: // a
            std::cout << "AUTO Mode" << std::endl;
            while(true){
                    autoInput.run(serial,motors);
                    Sleep(1000);
            }
        break;
        default:
            printf("Unknown keyboard input in main menu: ascii = %d\n",(int) x);
        break;
    }
 }

DemoMode::DemoMode()
{

}



//void DemoMode::lowerFlexLeg(long &liftPosition, Motors &motors, long &flexPostion, long &flexsteps, long &liftsteps, SerialChannel &serial)
//{
//    std::string motor;
//    motorPosition(serial, motors);
//    liftPosition = y_steps;
//    liftsteps = -15000;
//    liftPosition = liftPosition + liftsteps;
//    flexPostion = z_steps;
//    flexsteps = 3000;
//    flexPostion = flexPostion + flexsteps;
//    downMove(liftsteps, motor);
//    flexMoveDown(flexsteps, motor);
//    std::cout << "Flexing Leg..." << std::endl;
//    motorPosition(serial, motors);
//    while(y_steps != liftPosition && z_steps != flexPostion){
//        motorPosition(serial, motors);
//    }
//}

//void DemoMode::flexLeg(long &flexsteps, Motors &motors, long &flexPostion, SerialChannel &serial)
//{
//    std::string motor;
//    motorPosition(serial, motors);
//    flexPostion = z_steps;
//    flexsteps = 14500;
//    flexPostion = flexPostion + flexsteps;
//    flexMoveRight(flexsteps, motor);
//    std::cout << "Raising Leg..." << std::endl;
//    motorPosition(serial, motors);
//    while(y_steps != flexPostion){
//        motorPosition(serial, motors);
//    }
//}


//void DemoMode::downMove(long &steps, std::string &motor)
//{
//    std::cout << "Move leg up" << std::endl;
//    l_steps = steps;
//    motor = "l";
//    std::cout << "motor = " << motor <<
//                 " steps = " << l_steps <<
//                 std::endl;
//}
//void DemoMode::flexMoveDown(long &steps, std::string &motor)
//{
//    std::cout << "Move leg up" << std::endl;
//    s_steps = steps;
//    motor = "f";
//    std::cout << "motor = " << motor <<
//                 " steps = " << s_steps <<
//                 std::endl;
//}
//void DemoMode::flexMoveRight(long &steps, std::string &motor)
//{
//    std::cout << "Move leg up" << std::endl;
//    s_steps = steps;
//    motor = "f";
//    std::cout << "motor = " << motor <<
//                 " steps = " << s_steps <<
//                 std::endl;
//}

void DemoMode::upMove(SerialChannel serial, Motors motors, long &steps, std::string &motor)
{
    std::cout << "Move leg up" << std::endl;
    steps = 15000;
    l_steps = steps;
    motor = "l";
    std::cout << "motor = " << motor <<
                 " steps = " << l_steps <<
                 std::endl;
    sendCommandToChannel(serial, motors);
}

void DemoMode::raiseLeg(SerialChannel serial, Motors motors)
{
    std::string motor;
    motorPosition(serial, motors);
    std::cout << "yposition is = " << ysteps << std::endl;

    long liftPosition = ysteps;
    long liftsteps = l_steps;
    liftPosition = liftPosition + liftsteps;
    std::cout << "Lift position 0 is = " << liftPosition << std::endl;

    upMove(serial, motors, steps, motor);
    std::cout << "Raising Leg..." << std::endl;
    motorPosition(serial, motors);
    while(ysteps != liftPosition){
        motorPosition(serial, motors);
        std::cout << "Lift position end is = " << liftPosition << std::endl;
    }
    exit(0);
}
void DemoMode::resetSteps()
{
    l_steps = 0;
    s_steps = 0;
}

void DemoMode::sendCommandToChannel(SerialChannel serial, Motors motors)
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

void DemoMode::DemoMove(SerialChannel serial, Motors motors)
{
    long flexsteps = 0;
    long liftsteps = 0;
    long liftPosition = 0;
    long flexPostion = 0;

    resetSteps();//reset y_steps and z_steps

    //Raise leg
    std::cout << "doing raiseleg now" << std::endl;
    raiseLeg(serial, motors);
    //lower leg and flex slightly
//    lowerFlexLeg(liftPosition, motors, flexPostion, flexsteps, liftsteps, serial);
    //flex leg to 90 degrees
//    flexLeg(flexsteps, motors, flexPostion, serial);
//    resetFlexMove(steps, motor);
//    resetLiftMove(steps, motor);
}

void DemoMode::motorPosition(SerialChannel serial, Motors motors)
{
    serial.tx(motors.commandFactory("p", 0));
    std::string data = serial.rx();
    //Find the unique identifier cpos
    std::size_t found = data.find("cpos");
    long x_steps = 0, y_steps = 0 ,z_steps = 0;
    char a;
    char cposIdentify[100];
    if (found!=std::string::npos){ //no 'cpos' substring in data
        sscanf(data.c_str(),"%c %s %ld %ld %c",&a, &cposIdentify, &z_steps, &y_steps,&a);
        std::cout << "Arduino x_steps = " << x_steps << std::endl;
        std::cout << "Arduino z_steps = " << z_steps << std::endl;
        std::cout << "Arduino y_steps = " << y_steps << std::endl;
        std::cout << "---" << std::endl;
        xsteps = x_steps, ysteps = y_steps ,zsteps = z_steps;
        std::cout << "---" << std::endl;
    }
 }
void DemoMode::resetFlexMove(long &steps, std::string &motor)
{
    std::cout << "Reset Flex Axis" << std::endl;
    motor = "f";
    steps = -19000;
    s_steps = steps;
    std::cout << "flex motor = " << motor <<
                 " reset flex steps = " << s_steps <<
                 std::endl;
}
void DemoMode::resetLiftMove(long &steps, std::string &motor)
{
    std::cout << "Reset Lift Axis" << std::endl;
    motor = "l";
    steps = -19000;
    l_steps = steps;
    std::cout << "lift motor = " << motor <<
                 " lift reset steps = " << l_steps <<
                 std::endl;
}
void DemoMode::StartDemo(int &m, long &steps, std::string &motor, SerialChannel serial, Motors motors)
{
    switch(m) {
        case 113: // q
            std::cout << "Exiting Program" << std::endl;
            exit(0);
        case 13: // enter
        std::cout << "Start Demo Program" << std::endl;

            ready_to_move = true;
            motor = "l";
            resetLiftMove(steps, motor);
            motor = "f";
            resetFlexMove(steps, motor);
 //           motorPosition(serial, motors);
            std::cout << "---" << std::endl;
            std::cout << "---" << std::endl;
            while(zsteps != 0 && ysteps != 0 && xsteps != 0){
                std::cout << "x'steps = " << xsteps << std::endl;
                std::cout << "z'steps = " << zsteps << std::endl;
                std::cout << "y'steps = " << ysteps << std::endl;
                std::cout << "Please Wait..." << std::endl;
                motorPosition(serial, motors);
                Sleep(3000);
            }
            std::cout << "x-steps = " << xsteps << std::endl;
            std::cout << "z-steps = " << zsteps << std::endl;
            std::cout << "y-steps = " << ysteps << std::endl;
            DemoMove(serial, motors);
            break;
        default:
            printf("Unknown keyboard input in manual mode: ascii = %d\n",(int) m);
            break;
    }
}

void DemoMode::run(SerialChannel serial, Motors motors)
{
    if(kbhit()){
        stop_auto = true;
//        std::string motors;
        std::string motor;
        long steps;
        std::cout << "Press ESC to exit demo" << std::endl;
        int m = getch();
        if(m == 113 || m == 13)
        {
            std::cout << "We made it" << std::endl;
            StartDemo(m, steps, motor, serial, motors);

        }else{
            std::cout << "please press ENTER to start demo or q to quit" << std::endl;
        }
    }
}
