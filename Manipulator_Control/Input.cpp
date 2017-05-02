#include "Input.h"

ManualInput::ManualInput()
{

}
void ManualInput::upKey(std::string &motor, long &steps, long &velocity)
{
    std::cout << "up" << std::endl;
    motor = "l";
    steps = 100;
    velocity = 2000;
    l_velocity = velocity;
    l_steps += steps;
    std::cout << "motor = " << motor <<
                 " steps = " << l_steps <<
                 " velocity = " << l_velocity <<
                 std::endl;
}
void ManualInput::downKey(std::string &motor, long &steps, long &velocity)
{
    std::cout << "down" << std::endl;
    motor = "l";
    steps = -100;
    velocity = 2000;
    l_steps += steps;
    l_velocity = velocity;
    std::cout << "motor = " << motor <<
                 " steps = " << l_steps <<
                 " velocity = " << l_velocity <<
                 std::endl;
}
void ManualInput::rightKey(std::string &motor, long &steps, long &velocity)
{
    std::cout << "right" << std::endl;
    motor = "f";
    steps = 500;
    velocity = 4000;
    s_steps += steps;
    s_velocity = velocity;
    std::cout << "motor = " << motor <<
                 " steps = " << s_steps <<
                 " velocity = " << s_velocity <<
                 std::endl;
}
void ManualInput::leftKey(std::string &motor, long &steps, long &velocity)
{
    std::cout << "left" << std::endl;
    motor = "f";
    steps = -500;
    velocity = 4000;
    s_steps += steps;
    s_velocity = velocity;
    std::cout << "motor = " << motor <<
                 " steps = " << s_steps <<
                 " velocity = " << s_velocity <<
                 std::endl;
}
void ManualInput::checkArrowKeys(std::string &motor, long &steps, long &velocity)
{
    x=_getch();

    switch(x) {
        case 72:    // key up
            upKey(motor, steps, velocity);
            break;
        case 80:    // key down
            downKey(motor, steps, velocity);
            break;
        case 77:    // key right
            rightKey(motor, steps, velocity);
            break;
        case 75:    // key left
            leftKey(motor, steps, velocity);
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
                 " steps @" ;
    std::cout << s_velocity <<
                 " steps/sec" <<
                 std::endl;
    std::cout << "l_motor = " <<
                 l_steps <<
                 " steps @";
    std::cout << l_velocity <<
                 " steps/sec" <<
                 std::endl;
    ready_to_move = true;
}
void ManualInput::resetSteps()
{
    l_steps = 0;
    s_steps = 0;
    l_velocity = 100;
    s_velocity = 100;
}
void ManualInput::sendCommandToChannel(SerialChannel serial, Motors motors)
{

    if(ready_to_move == true){
        serial.tx(motors.commandFactory("f", s_steps, s_velocity));
        std::cout << "Response...";
        std::string stdStringData = serial.rx();
        std::cout << "data = " << stdStringData << std::endl;

        serial.tx(motors.commandFactory("l", l_steps, l_velocity));
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
void ManualInput::checkOtherKeys(Motors motors, SerialChannel serial, std::string &motor)
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
            sendCommandToChannel(serial, motors);
            break;
        case 105: // i
            std::cout << "Manual input. Usage: motor steps velocity" << std::endl;
            char line[100];
            char sscanf_motor[100];
            long manual_steps;
            long manual_velocity;
            std::cin.getline( line, 100,'\n' );
            sscanf(line, "%s %ld %ld", &sscanf_motor, &manual_steps, &manual_velocity);
            motor = (std::string)sscanf_motor;
            if(motor.compare("f") == 0){
                s_steps = manual_steps;
                s_velocity = manual_velocity;
            }else if(motor.compare("l") == 0){
                l_steps = manual_steps;
                l_velocity = manual_velocity;
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
   //     stop_auto = true;
        std::string motor;
        long steps;
        long velocity;
        x = getch();
        if(x==0 || x==-32)
        {
            ready_to_move = false;
            checkArrowKeys(motor, steps, velocity);
        }else{
            checkOtherKeys(motors, serial, motor);
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
    serial.tx(motors.commandFactory("p", 0, 0));
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
    long s_auto_velocity = 100;
    long l_auto_velocity = 100;

    std::cout << "--->Flex steps to arduino = " << s_auto_steps << std::endl;
    std::cout << "--->Lift steps to arduino = " << l_auto_steps << std::endl;

    serial.tx(motors.commandFactory("f", s_auto_steps, s_auto_velocity));
    std::cout << "Arduino Response...";
    std::string stdStringData = serial.rx();
    std::cout << "f auto data = " << stdStringData << std::endl;

    serial.tx(motors.commandFactory("l", l_auto_steps, l_auto_velocity));
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
        std::cout << "Enter password: ";
        password = "";
        while((ch = _getch()) != ENTER)
        {
            password += ch;
            std::cout << '*';
        }
        std::cout << std::endl;
        if (password == demoPassword){
            std::cout << "MANIPULATOR DEMO MODE" << std::endl;
            std::cout << "\tUsage: " << std::endl
                      << "\t (i)  Use 'q' to quit demo program" << std::endl
                      << "\t (ii) Press ENTER to continue with the demonstation" << std::endl
                      << std::endl;        }
            while(password == demoPassword){
                    demoMode.run(serial,motors);
                    Sleep(1000);
            }
            std::cout << "Close but not there yet! - " ;
             break;
        case 109: // m
            if (countm < 1){
                std::cout << "Manual Input Mode" << std::endl;
                std::cout << "Usage: " << std::endl
                          << "\t Use Left and Right arrow key for slider motor" << std::endl
                          << "\t Use Up and Down arrow key for Lift motor" << std::endl
                          << "\t Use 'i' key for manual step entry (motor (f or l) steps)" << std::endl
                          << "\t Use 'r' key for step summary" << std::endl
                          << "\t Use 'Enter' key to send steps to motors" << std::endl
                          << std::endl;
            }
            std::cout << "Enter password: ";
            password = "";
            while((ch = _getch()) != ENTER)
            {
                password += ch;
                std::cout << '*';
            }
            std::cout << std::endl;
            if (password == manualPassword){
                std::cout << "Please enter manual command: ";
            }
            while(password == manualPassword){
                manualInput.run(serial,motors);
   //             if(!manualInput.stop_auto){
   //                 autoInput.run(serial,motors);
   //         }
           }
           std::cout << "Good try! - Nearly fooled me! - " ;
           countm += 1;
 //          std::cout << countm << std::endl;
        break;
        case 97: // a
//            std::cout << "AUTO Mode" << std::endl;
            std::cout << "Enter Auto password: ";
            password = "";
            while((ch = _getch()) != ENTER)
            {
                password += ch;
                std::cout << '*';
            }
            std::cout << std::endl;
            if (password == autoPassword){
                std::cout << "Entering Auto Mode: " << std::endl;
                Sleep(2000);
            }
                while(password == autoPassword){
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
void DemoMode::flexLiftCommand(long newFlexSteps, long newLiftSteps, SerialChannel serial, Motors motors)
{
    motorPosition(serial, motors);//read existing position data into xsteps, ysteps and zsteps
    double liftPosition = ysteps;
    double flexPosition = zsteps;
    s_steps = newFlexSteps+flexPosition;// Use absolute position in UNO
    l_steps = newLiftSteps+liftPosition;// Use absolute position in UNO

    std::cout << "--s_steps read from UNO = " << s_steps << std::endl;
    std::cout << "--l_steps read from UNO = " << l_steps << std::endl;
    std::cout << "--New flex Steps to be added = " << newFlexSteps << std::endl;
    std::cout << "--New lift Steps to be added = " << newLiftSteps << std::endl;
    std::cout << "" << std::endl;

    sendCommandToChannel(serial, motors);
}
void DemoMode::flexLiftControl(long newFlexSteps, long newLiftSteps, SerialChannel serial, Motors motors)
{
    int countsteps = 0;
    //CHECK INITIAL POSITION OF FLEX and LIFT MOTOR
    motorPosition(serial, motors);//read position data into xsteps, ysteps and zsteps
    std::cout << "" << std::endl;
    std::cout << "Starting flex and Lift position is = " << zsteps <<"," <<ysteps << std::endl;

    //COMMAND TO FLEX and LIFT MOTORS
    flexLiftCommand(newFlexSteps, newLiftSteps,serial, motors);
    std::cout << "Moving Leg..." << std::endl;

    motorPosition(serial, motors);// read new position of flex and lift
    if (resetMove == TRUE){
        std::cout << "RESET IN PROGRESS" << std::endl;
        while(xsteps != 0 || ysteps != 0 || zsteps != 0){
             if(countsteps == 0){
            std::cout << "Steps for x motors not zero = " << xsteps << std::endl;
            std::cout << "Steps for y motors not zero = " << ysteps << std::endl;
            std::cout << "Steps for z motors not zero = " << zsteps << std::endl;
            }
            countsteps += 1;
            motorPosition(serial, motors);
            std::cout << "------>Steps (x, y, z) = " << xsteps << ", "<< ysteps << ", "<< zsteps << std::endl;
        }
        std::cout << "**Steps for all motors now zero (x, y, z) = " << xsteps << ", "<< ysteps << ", "<< zsteps << std::endl;
        resetMove = FALSE;
    }
    else{
        std::cout << "NOT RESET" << std::endl;    std::cout << "--s_steps = " << s_steps << std::endl;
        std::cout << "" << std::endl;
        while(ysteps != l_steps || zsteps != s_steps){
            motorPosition(serial, motors);//reading position as lift motor do steps
            std::cout << "------>Steps (x, y, z) = " << xsteps << ", "<< ysteps << ", "<< zsteps << std::endl;
        }
    }
}
void DemoMode::resetSteps()
{
    l_steps = 0;
    s_steps = 0;
}
void DemoMode::sendCommandToChannel(SerialChannel serial, Motors motors)
{
        serial.tx(motors.commandFactory("f", s_steps, s_velocity));//write steps to port
        std::cout << "-Send steps to flex motor - ";
        Sleep(5);
        std::string stdStringData = serial.rx();
        std::cout << "Response ... " << stdStringData << std::endl;

        serial.tx(motors.commandFactory("l", l_steps, l_velocity));
        std::cout << "-Send steps to lift motor - ";
        Sleep(5);
        std::string stdStringData1 = serial.rx();
        std::cout << "Response... " << stdStringData1 << std::endl;
        std::cout << "**Motor commands sent" << std::endl;
        std::cout << "" << std::endl;
}

void DemoMode::moveControl(SerialChannel serial, Motors motors)
{
    resetSteps();//reset l_steps and s_steps

    // FORMAT: flexLiftControl(FLEX steps, LIFT steps, serial, motors);

    std::cout << "           ----------MANIPULATOR RESET PHASE-----------" << std::endl;
    resetMove = TRUE;
    l_velocity = 6400;
    s_velocity = 12800;
    flexLiftControl(-19000, -19000, serial, motors);

    std::cout << "           ---------------LIFX LEG PHASE---------------" << std::endl;
    resetMove = FALSE;
    l_velocity = 12800;
    s_velocity = 12800;
    flexLiftControl(3, 10, serial, motors);

    std::cout << "           ---------LOWER AND FLEX LEG PHASE-----------" << std::endl;
    resetMove = FALSE;
    l_velocity = 12800;
    s_velocity = 2000;
    flexLiftControl(7, -5, serial, motors);

    std::cout << "           ---------------FLEX LEG PHASE---------------" << std::endl;
    resetMove = FALSE;
    l_velocity = 0;
    s_velocity = 12800;
    flexLiftControl(10, 0, serial, motors);
}
void DemoMode::motorPosition(SerialChannel serial, Motors motors)
{
    serial.tx(motors.commandFactory("p", 0, 0));
    std::string data = serial.rx();
    //Find the unique identifier cpos
    std::size_t found = data.find("cpos");
    long x_steps = 0, y_steps = 0 ,z_steps = 0;
    char a;
    char cposIdentify[100];
    if (found!=std::string::npos){ //no 'cpos' substring in data
        sscanf(data.c_str(),"%c %s %ld %ld %c",&a, &cposIdentify, &z_steps, &y_steps,&a);
        xsteps = -x_steps, ysteps = -y_steps ,zsteps = -z_steps;
        std::cout << "------>Manipulator Postion from UNO (x-N/A, y-lift, z-flex) = " << xsteps << ", "<< ysteps << ", "<< zsteps << std::endl;
        std::cout << "                                             " << std::endl;
    }
 }
void DemoMode::StartDemo(int &m, SerialChannel serial, Motors motors)
{
    std::string motor;
    switch(m) {
        case 113: // q
            std::cout << "Exiting Program" << std::endl;
            exit(0);
        case 13: // enter
        std::cout << "                                  Demo Program" << std::endl;
            moveControl(serial, motors);
            exit(0);
            break;
        default:
            printf("Unknown keyboard input in manual mode: ascii = %d\n",(int) m);
            break;
    }
}
void DemoMode::run(SerialChannel serial, Motors motors)
{
    if(_kbhit()){
        stop_auto = true;
        std::cout << "Press ESC to exit demo" << std::endl;
        int m = _getch();
        if(m == 113 || m == 13)
        {
            StartDemo(m, serial, motors);

        }else{
            std::cout << "please press ENTER to start demo or q to quit" << std::endl;
        }
    }
}
