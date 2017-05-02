#include <AccelStepper.h>

unsigned long baud = 115200;
const int led_pin = 13; // 13 = Teensy 3.X
const int led_on = HIGH;
const int led_off = LOW;

//const int freq = //counter for resetting PWM output. Desired Frequency / interupt frequency.
long int duty = 0; //# of PWM cycles output is high
char uartind=0;
char uartbuff[100];
float *packet;
unsigned char inputel=0;

//VARIABLES

  //Define variables to switch pins
  int FRsw = A1; //variable for flex R switch pin
  int FLsw = A0; //variable for flex L switch pin
  int LRsw = A2; //variable for Lift T switch
  int LLsw = A3; //variable for Lift B switch
  int TRsw = 10; //variable for Thight R switch
  int TLsw = 9; //variable for Thigh L switch

  //Defines Variables for state of switches - mainly for debouncing swithes
  int FRswVal = LOW; //variable to store R switch state for flex motor
  int FLswVal = LOW; //variable to store L switch state for flex motor
  int LRswVal = LOW; //variable to store R switch state for Lift motor
  int LLswVal = LOW; //variable to store L switch state for Lift motor

  //Defines Variables for state of steps and position
  long F_STEPS ;                //variable to store steps required to move
  long F_SPEED = 100;                //variable to store steps required to move
  long L_SPEED = 100;                //variable to store steps required to move
  long F_PositionReset;         //Set position to new current position once switch is hit
  long F_StepsRemaining;        //steps left to finish input
  long F_ActualCurrentPostion;  //variable to store actual position from 0
  long L_STEPS ;                //variable to store steps required to move
  long L_PositionReset;         //Set position to new current position once switch is hit
  long L_StepsRemaining;        //steps left to finish input
  long L_ActualCurrentPostion;  //variable to store actual position from 0

  //Set variables to debounce switches
  int count = 0;
  // Switch debounce Variables:
  int buttonState;                      // the current reading from the input pin
  int lastButtonState = HIGH;           // the previous reading from the input pin
  unsigned long lastDebounceTime = 0;   // the last time the output pin was toggled
  unsigned long debounceDelay = 50;     // the debounce time; increase if the output flickers

//DEFINE A STEPPER AND THE PINS IT WILL USE
  /* COMMAND: AccelStepper flexMotor(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
  AccelStepper example(1, Dir, PWM);
  1: unknown but needed
  8: Dir 
  9: Pulse
  */
  AccelStepper flexMotor(1, 3, 4);  //PUL:Pin 3, DIR:Pin 4
  AccelStepper liftMotor(1, 5, 8);  //PUL:Pin 5, DIR:Pin 8
  AccelStepper thighMotor(1, 6, 12);//PUL:Pin 6, DIR:Pin 12

//COMMAND TO MOVE MOTOR
  //  $ l 500 1200 # -> $ (start char) s (motor char) 4000 (number of steps) # (terminating char)
  
void setup()
{  
  //SETUP SERIAL COMMUNICATIONS WITH PC
    Serial.begin(baud);	// USB, communication to PC or Mac              //Initialise UART tranciver hardware  
 
  //SETUP MOTOR PARAMETERS
    flexMotor.setMaxSpeed(100); //approx. 2.5mm/sec - default = 12800
    flexMotor.setAcceleration(1500); // Default = 4000
    flexMotor.move(0); // set initial speed to zero for safety
    liftMotor.setMaxSpeed(100); //approx. 2.5mm/sec- default = 6800
    liftMotor.setAcceleration(1500);
    liftMotor.move(0); // set initial speed to zero for safety

  //ENABLE OR DISABLE MOTOR DRIVERS
    //Setup enable switch pins as inputs and set high via external pullup resistor - will work even is arduino is not working
    //Can test these and take action if required
    pinMode(2, INPUT);// Input for enable pin - use external resistor to pull it high - switch can ground it (used INPUT_PULLUP in past to get it high)
    pinMode(7, INPUT);// Input for enable pin - use external resistor to pull it high - switch can ground it (used INPUT_PULLUP in past to get it high)
    pinMode(11, INPUT);// Input for enable pin - use external resistor to pull it high - switch can ground it (used INPUT_PULLUP in past to get it high)

  //LIMIT SWITCHES TO RESTRICT MOVEMENT AND DETERMINE ABSOLUTE POSITION
    //Setup limit switch pins as inputs and set high via pullup resistor - will work even is arduino is not working
    pinMode(A1, INPUT_PULLUP);//Setup flex limit RIGHT switch to Stop motor
    pinMode(A0, INPUT_PULLUP);//Setup flex limit LEFT switch to Stop motor
    pinMode(A2, INPUT_PULLUP);//Setup lift limit TOP switch to Stop motor
    pinMode(A3, INPUT_PULLUP);//Setup lift limit BOTTOM switch to Stop motor
    
  // SET INITIAL POSITION FOR MOTORS
    flexMotor.setCurrentPosition(19000); //set position to 0 where ever the motor start - will change that once a switch is reached
    liftMotor.setCurrentPosition(19000); //set position to 0 where ever the motor start - will change that once a switch is reached

} // END OF SETUP

void loop() {
  
  char *pnt;
  long data;     //S1 S2 D1 D2 D3
  long velocity;     
  unsigned char uartstr[10];
	unsigned char a,c,dtr, motor;
  unsigned char b=0;
	static unsigned char prev_dtr = 0;
  
  if (Serial.available()) { //Data arrived from host
     uartbuff[uartind] = Serial.read();
      if (uartbuff[0]=='$'||uartbuff[0]==0x42){//Detect '$' character in start byte, proceed to fill buffer with subsequent arrivals
          if (uartbuff[uartind]=='#'){//Detect '#' termination character at end of packet, proceed to extracting and processing data.  //Read packet. //Read packet. //Read packet. //Read packet.
             //Serial.println("Command Accepted"); 
             sscanf(uartbuff,"%c %c %ld %ld %c",&a, &motor, &data, &velocity, &a);
             if(motor == 'l'){
                char data_packet[100];
                sprintf(data_packet, "# %ld %ld $", data, velocity);
                Serial.print(data_packet);
                L_STEPS = data; //Read steps in global variable                char data_packet[100];
                L_SPEED = velocity;
                liftMotor.setMaxSpeed(L_SPEED);
                liftMotor.moveTo(-data);//move to data steps as specified via terminal or program
              }  
             if(motor == 'f'){
                char data_packet[100];
                sprintf(data_packet, "# %ld %ld $", data, velocity);
                Serial.print(data_packet);
                F_SPEED = velocity;
                flexMotor.setMaxSpeed(F_SPEED);
                F_STEPS = data; //Read steps in global variable
                flexMotor.moveTo(-data);//move to data steps as specified via terminal or program
              }  
              if(motor == 'p'){ //to send position to c++ interface
                  F_ActualCurrentPostion = flexMotor.currentPosition();
                  L_ActualCurrentPostion = liftMotor.currentPosition();
                  char data_packet[100];
                  sprintf(data_packet, "# cpos %ld %ld $", F_ActualCurrentPostion, L_ActualCurrentPostion);
                  Serial.print(data_packet);               
              }
              if(motor == 'L'){
                char data_packet[100];
                sprintf(data_packet, "# %ld %ld $", data, velocity);
                Serial.print(data_packet);
                L_STEPS = data; //Read steps in global variable                char data_packet[100];
                L_SPEED = velocity;
                liftMotor.setMaxSpeed(L_SPEED);
                liftMotor.moveTo(-data);//move to data steps as specified via terminal or program
              }  
             if(motor == 'F'){
                char data_packet[100];
                sprintf(data_packet, "# %ld %ld $", data, velocity);
                Serial.print(data_packet);
                F_SPEED = velocity;
                flexMotor.setMaxSpeed(F_SPEED);
                F_STEPS = data; //Read steps in global variable
                flexMotor.moveTo(-data);//move to data steps as specified via terminal or program
              }   
                data = 0;
                inputel=0;
                uartbuff[0]=' ';              //clear start of uart buffer to escape uart reception mode
                uartind=0;
           }
           else{
              if(uartbuff[uartind]==' '){inputel++;}  //Detected delimiting character ' ', counter used to determine command type 
                 uartind++;                              //Uart packet length
                 if(uartind==30){                        //arbritrary max buffer length, can be extended if needed
                   Serial.println('string too long (Max:30)');
                   uartind=0;
                   inputel=0;
                   uartbuff[0]=' ';
                 }
           }//end of checking for terminating character
              
     }//End checking for start character
      return;
	 }//End of serial communications

//MOTOR MOVEMENT AND SWITCH CONTROL
  //check if limits were reached
  FRswVal = digitalRead(FRsw);
  FLswVal = digitalRead(FLsw);
  LRswVal = digitalRead(LRsw);//top 
  LLswVal = digitalRead(LLsw);//bottom

  F_StepsRemaining = flexMotor.distanceToGo();
  L_StepsRemaining = liftMotor.distanceToGo();

  if(FRswVal == HIGH && FLswVal == HIGH && LRswVal == HIGH && LLswVal == HIGH) {
    buttonState = HIGH;
    lastDebounceTime = millis();
  }
    else {
      // reset the debouncing timer
      if ((millis() - lastDebounceTime) > debounceDelay) {  //millis is the time since the program started to run
        // whatever the reading is at, it's been there for longer
        // than the debounce delay, so take it as the actual current state:
        // if the button state has changed:
        buttonState = LOW;
      }
    }
  // what to do if switch is HIGH (OPEN) - not pressed
  if (buttonState == HIGH) {
         count = 0;
    }            
  // what to do if switch has gone LOW (CLOSED) - something has pressed the logical switch
  if (buttonState == LOW) {
        count = count + 1;  //Keep counting while switch is pressed
        if(count == 5) {
            count = 2; //ensure count stay at a low number - use counts to debounce switch
          }
   }
//ACTION WHILE SWITCH IS PRESSED   
  if(count == 1) {  //take action if count is 1 - means switch was pressed
    F_ActualCurrentPostion = flexMotor.currentPosition();
    L_ActualCurrentPostion = liftMotor.currentPosition();

   //Left switch   
      if(FLswVal == LOW){ //A0 - Left switch of flex motor
        flexMotor.setSpeed(0);
        flexMotor.setCurrentPosition(0); //set position to 0 once the Right switch is reached
      }
      if(LLswVal == LOW){ //A3 - Bottom switch of lift motor
        liftMotor.setSpeed(0);
        liftMotor.setCurrentPosition(0); //set position to 0 once the Right switch is reached
      }
   //Right switches - Stop motors   
     if(FRswVal == LOW){  //A1
       flexMotor.setSpeed(0);
          /*  CODE TO CHANGE DIRECTION - NOT USED
              F_StepsRemaining = -F_StepsRemaining;
              flexMotor.setCurrentPosition(-F_StepsRemaining); //reset distance to go to 0 once a switch is reached
              delay(100);
              flexMotor.move(F_StepsRemaining);//change direction when at left switch
        */
      }
      if(LRswVal == LOW){ //A2
        flexMotor.setSpeed(0);
      }
  }//end action while count = 1
  
flexMotor.run(); //steps only one step at a time
liftMotor.run();
}
