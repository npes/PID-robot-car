#include <stdio.h>
#include <SoftwareSerial.h>  

/*This program drives the wheels of the robot forward and uses the tacometers to count the rotations in ticks
The ticks is summed in the variables int stateLeftTaco and int stateRightTaco. Speed is controlled with the variables slowL 
and slowR*/

/* ======================================= Pins setup =========================================== */

int bluetoothTx = A0;  // TX-O pin of bluetooth
int bluetoothRx = A1;  // RX-I pin of bluetooth
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

//Taco meters pins
  const int leftTacho=A4;
  const int rightTacho=A5;

// sensors pins
  int center = 8;         //hvid PB0
  int centerLeftS = 7;    //lilla PD7
  int centerRightS = 13;  //grøn PB5
  int leftS = 4;          //blå PD4
  int rightS = 2;         //brun PD0

// motor pins
  int motorleftSA=11; // pin 11 på arduino forbundet til pin 2 på motor driver
  int motorrightSA=10; // pin 10 på arduino forbundet til pin 10 på motor driver
  int motorleftSB=9; // pin 9 på arduino forbundet til pin 7 på motor driver
  int motorrightSB=3; // pin 3 på arduino forbundet til pin 15 på motor driver
  
  int motorleftSSpeed=5; // pin 5 på arduino forbundet til pin 1 på motor driver
  int motorrightSSpeed=6; // pin 6 på arduino forbundet til pin 9 på motor driver

/* ======================================= Pins setup End =========================================== */

/* ====================================== global variables ==========================================*/ 

//variables for PID functions 
  const float Kp_value = 0.5;
  const float Ki_value = 0.1;
  const float Kd_value = 0.0010;

  // starting pwm for both motors
  short int old_DutyL = 80; 
  short int old_DutyR = 80; 

  // value in either micro sec 
  float EXT_SetPoint = 60;
   
  // error value
  float EPchangeL = 0;
  float EPchangeR = 0;

  // Freq. for each tacho
  float TachoLeftFreq = 0;
  float TachoRightFreq = 0;

  //start and stop from bluetooth
  int go=1; //used to determine start or stop from bluetooth buttons. 1=start 0=stop
  
/* ==================================== global variables End =========================================*/ 
  
// ======================================= Function prototypes =======================================
  void tachoFunction(); // function to check tachos, the time periode / frequencies, used for checking error 
  void ReadSensors();   // use for line following
  // PID function for motor control. requires pwm input as starting point
  int PIDFunctionL(int PWMSignal, int InputSetPoint, float SensorError); 
  int PIDFunctionR(int PWMSignal, int InputSetPoint, float SensorError);

  // bluetooth
  int func_BT_init (void); //initializes bluetooth, call it in setup
  int func_BTGraph(int HertzL, int HertzR, int SetPoint); //prints 3 variables to a BlueTooth graph
  int func_BTPIDLValues(float P, float I, float D); //Prints PID variables to BlueTooth console 
  int func_BTPIDRValues(float P, float I, float D); //Prints PID variables to BlueTooth console 
  int func_BTreadbuttons (void); //Reads BlueTooth buttons when called
// ===================================== Function prototypes End =====================================

/* ****************************************** Setup Start *********************************************** */   
void setup() {
/* ====================================== pin direction setup ====================================== */
  // sensors  
  pinMode (center, INPUT);
  pinMode (centerLeftS, INPUT);
  pinMode (centerRightS, INPUT);
  pinMode (leftS, INPUT);
  pinMode (rightS, INPUT);

  // motors
  pinMode (motorleftSA, OUTPUT);
  pinMode (motorrightSA, OUTPUT);
  pinMode (motorleftSB, OUTPUT);
  pinMode (motorrightSB, OUTPUT);
  pinMode (motorleftSSpeed, OUTPUT);
  pinMode (motorrightSSpeed, OUTPUT);

  //Tacho meters
  pinMode(rightTacho, INPUT);
  pinMode(leftTacho, INPUT);

/* ====================================== pin direction setup End ====================================== */

  Serial.begin(9600);
  func_BT_init(); //initialize bluetooth and clear graph

// ====================================== moter speed settings ======================================

    digitalWrite(motorleftSA, 1);
    digitalWrite(motorleftSB, 0);
    
    digitalWrite(motorrightSA, 1);
    digitalWrite(motorrightSB, 0);
// ==================================== moter speed settings End =====================================  
}
/* ****************************************** Setup end ************************************************* */ 

/* **************************************** void loop start ********************************************* */
void loop() {
  //Serial.print(EXT_SetPoint);
  //Serial.print(" "); 
  func_BTreadbuttons (); //Read BlueTooth buttons
  if (go==1) //start has been pressed
  {
    // drive forward, using PWM
    analogWrite(motorleftSSpeed, old_DutyL);
    analogWrite(motorrightSSpeed, old_DutyR);
    
    // Read values from tacho, store freq. value for both in a variable for each
    ReadTacho();   
  
    // Read sensors to follow line, and change error for PID accordingly to the line
    ReadSensors();
    // calls PID functions for both motors and store returned result for use next time functuion is called
    
    old_DutyL = PIDFunctionL(old_DutyL , EXT_SetPoint, EPchangeL);
    old_DutyR = PIDFunctionR(old_DutyR , EXT_SetPoint, EPchangeR);
    func_BTGraph(TachoLeftFreq, TachoRightFreq, EXT_SetPoint); //write Hertz and setpoint values to bluetooth graph
  }
  else 
  {
    analogWrite(motorleftSSpeed, 0);
    analogWrite(motorrightSSpeed, 0);
  }
}
/* ****************************************** void loop end ********************************************* */

/*===================================== functions ======================================*/
/* ************************************ PID functions ********************************* */
  // ================================== PID function L ================================
  int PIDFunctionL(int PWMSignal, int InputSetPoint, float SensorError){
    
    short int Error;    // error value taco left
    float PL;            // Holds the calculated Proportional value for taco left
    float IL;            // integral for left taco
    short int I_error;  // Holds the calculated Integral value for left taco
    float DL;            // Holds the calculated Differential value for left taco
    short int D_error;  // Holds the derivative error for left taco
    int SetPoint = InputSetPoint; 
    short int old_PWM = PWMSignal; // store function input (pwm) for use in function
    short int New_PWM = 0;    
    
    // Error of Set point vs FB for left taco
    Error = (SetPoint - TachoLeftFreq);  // setpoint is wished value, FB is output from taco
    Error += SensorError;           // adding error from line detetion
     
    // P factor and error
    PL = Kp_value * Error;
    
    // I_erro over time factor of the I term
    I_error += Error;
  
    // Set up a Integral windup protection. Set a max and a min level for the I error
  
    // I factor over time. Remember I is error over time. Which means if error is 1 over two iteration error is now 2
    IL = Ki_value * I_error;
  
    // D factor of the PID
    DL = Kd_value * (D_error - Error);
    
    // The New error becomes the error the D expects and if not the same it will adjust for it.
    D_error = Error;
  
    // the PID SUM Logic
    New_PWM = old_PWM + (PL + IL + DL);

    //load the new duty cycle to the variable of the old so you can adjust for it.
    old_PWM = New_PWM;

   // Implement a min max for your PWM value before you send it further.   
   if(old_PWM > 255)
    {
      old_PWM = 255;
    }
    else if(old_PWM < 60)
    {
       old_PWM = 60;
    } 
      
    analogWrite(motorleftSSpeed, old_PWM);
    func_BTPIDLValues(PL,IL,DL); //write P,I and D values to bluetooth console
    return old_PWM;
    
  }
  // ============================== PID function L End ================================

// PID function R
  int PIDFunctionR(int PWMSignal, int InputSetPoint, float SensorError){

    /*Serial.print("PWM: ");
    Serial.print(PWMSignal);
    Serial.print(", SetPoint: ");
    Serial.print(InputSetPoint);
    Serial.print(", Sensor Error: ");
    Serial.println(SensorError);*/
    
    short int Error;    // error value taco right
    float PR;        // Holds the calculated Proportional value for taco right
    float IR;        // integral for right taco
    short int I_error;  // Holds the calculated Integral value for right taco
    float DR;        // Holds the calculated Differential value for right taco
    short int D_error;  // Holds the derivative error for right taco
    int SetPoint = InputSetPoint; 
    short int old_PWM = PWMSignal; // store function input (pwm) for use in function
    short int New_PWM = 0;
    // Error of Set point vs FB for right taco
    Error = (SetPoint - TachoRightFreq); // setpoint is wished value, FB is output from taco
    
    // P factor and error
    PR = Kp_value * Error;
    
    // I_erro over time factor of the I term
    I_error += Error;
  
    // Set up a Integral windup protection. Set a max and a min level for the I error
  
    // I factor over time. Remember I is error over time. Which means if error is 1 over two iteration error is now 2
    IR = Ki_value * I_error;
  
    // D factor of the PID
    DR = Kd_value * (D_error - Error);
    // The New error becomes the error the D expects and if not the same it will adjust for it.
    D_error = Error;
  
    // the PID SUM Logic
    New_PWM = old_PWM + (PR + IR + DR + SensorError);
    
    //load the new duty cycle to the variable of the old so you can adjust for it.
    old_PWM = New_PWM;

    if(old_PWM > 255)
    {
      old_PWM = 255;
    }
    else if(old_PWM < 60)
    {
       old_PWM = 60;
    }
    
    // You can also implement a min max for your PWM value before you send it further.
    analogWrite(motorrightSSpeed, old_PWM);
    func_BTPIDRValues(PR,IR,DR); //write P,I and D values to bluetooth console
    return old_PWM;
    
}
/* ************************************ PID functions ********************************* */
/* ********************************* Tachometer readings ****************************** */
  void ReadTacho()
  {
    // Combined timePeriode for low and high in mili sec
    float LeftTimePeriod = 0;   
    float RightTimePeriod = 0;  
  
    //high and low time periode for left tacho 
    LeftTimePeriod += pulseIn(leftTacho, HIGH);
    LeftTimePeriod += pulseIn(leftTacho, LOW);
    TachoLeftFreq = 1000000/LeftTimePeriod;
    Serial.print(TachoLeftFreq);
    Serial.print(" ");
    
    //high and low time periode for right tacho
    RightTimePeriod += pulseIn(rightTacho, HIGH);
    RightTimePeriod += pulseIn(rightTacho, LOW);
    TachoRightFreq = 1000000/RightTimePeriod;
    Serial.println(TachoRightFreq);
 
  }
  
/* ********************************* Tachometer readings ****************************** */

/* ********************************* Sensor read start ******************************** */
  void ReadSensors()
  {
    int val = 0;

    // Sensor state - read sensors and store in variables
    int stateCenter = digitalRead (center);
    int stateCenterLeftS = digitalRead (centerLeftS);
    int stateCenterRightS = digitalRead (centerRightS);
    int stateLeftS = digitalRead (leftS);
    int stateRightS = digitalRead (rightS);
 

    // bitshift sensor states to make val represent the five sensors states in one binary value
    val = stateLeftS << 0;
    val += stateCenterLeftS << 1;
    val += stateCenter << 2;
    val += stateCenterRightS << 3;
    val += stateRightS << 4;

    switch (val)
    {
      case 4: // OOXOO
      //Serial.println("zero error");
        EPchangeR = 0;
        EPchangeL = 0;
      break;
  
      case 6: // OXXOO
      //Serial.println("Mini Left Error, speed up Right Wheel");
        EPchangeR = -2;
        EPchangeL = 6;
      break;
  
      case 2: // OXOOO
      //Serial.println("Small Left Error, speed up Right Wheel");
        EPchangeR = -4;
        EPchangeL = 8;
      break;
  
      case 3: // XXOOO
      //Serial.println("Medium Left Error, speed up Right Wheel");
        EPchangeR = -6;
        EPchangeL = 10;
      break;
    
      case 1: // XOOOO
      //Serial.println("Large Right Error, speed up Left Wheel");
        EPchangeR = -8;
        EPchangeL = 12;
      break;   
    
      case 12: // OOXXO
      //Serial.println("Mini Right Error, speed up Left Wheel");
        EPchangeR = 6;
        EPchangeL = -2;
      break;    
    
      case 8: // OOOXO
      //Serial.println("Small Right Error, speed up Left Wheel");
        EPchangeR = 8;
        EPchangeL = -4;
      break;
    
      case 24: // OOOXX
      //Serial.println("Medium Right Error, speed up Left Wheel");
        EPchangeR = 10;
        EPchangeL = -6;
      break;
  
      case 16: // OOOOX
      //Serial.println("Large Left Error, speed up Right Wheel");
        EPchangeR = 12;
        EPchangeL = -8;
      break;

      case 0:  // OOOOO
      //Serial.println("Line is gone, Go fetch");
        EPchangeR = 0;
        EPchangeL = 0;
      break;
    } 
  }
    
  
/* ********************************** Sensor read end  ******************************** */

/* ********************************** Bluetooth start ********************************* */
int func_BT_init (void) //initializes bluetooth, call it in setup
  {
    bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
    bluetooth.print("$");  // Print three times individually
    bluetooth.print("$");
    bluetooth.print("$");  // Enter command mode
    delay(100);  // Short delay, wait for the Mate to send back CMD
    bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
    // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
    bluetooth.begin(9600);  // Start bluetooth serial at 9600
    bluetooth.print("*GC*"); //Clear bluetooth graph
  }

int func_BTGraph(int HertzL, int HertzR, int SetPoint) //prints 3 variables to a BlueTooth graph
  {
    bluetooth.print("*G"); //select graph
    bluetooth.print(HertzL); //print HertzL to trace 1 
    bluetooth.print(".1"); 
    bluetooth.print(",");
    bluetooth.print(HertzR); //print HertzR to trace 2 
    bluetooth.print(".2"); 
    bluetooth.print(",");
    bluetooth.print(SetPoint); //print SetPoint to trace 3 
    bluetooth.print(".3*");
  }

int func_BTPIDLValues(float P, float I, float D) //Prints PID variables to BlueTooth console  
  {  
    bluetooth.print("*A"); //select console 1
    bluetooth.print("PL="); //print P to console
    bluetooth.println(P);

    bluetooth.print("IL="); //print I to console 1
    bluetooth.println(I);   

    bluetooth.print("DL="); //print D to console 1
    bluetooth.println(D);
    bluetooth.println(""); //insert a blank line to make it look pretty
    bluetooth.print("*"); // end of write to console 1
  }

int func_BTPIDRValues(float P, float I, float D) //Prints PID variables to BlueTooth console  
  {  
    bluetooth.print("*C"); //select console 1
    bluetooth.print("PR="); //print P to console
    bluetooth.println(P);

    bluetooth.print("IR="); //print I to console 1
    bluetooth.println(I);   

    bluetooth.print("DR="); //print D to console 1
    bluetooth.println(D);
    bluetooth.println(""); //insert a blank line to make it look pretty
    bluetooth.print("*"); // end of write to console 1
  }

int func_BTreadbuttons (void) //Reads BlueTooth buttons when called
  {  
    if(bluetooth.available())  // If the bluetooth app sent any characters
    {
      char btn=((char)bluetooth.read()); //store the received in btn 
      if (btn==0x53) //if char S is received 
      {
      bluetooth.print("*B"); //select console 2
      bluetooth.println("Start"); //print start to console 2
      bluetooth.print("*"); // end of write to console 1
      go=1; // set go to 1 to start the program
      }
        if (btn==0x52) //if char R is received
        {
        bluetooth.print("*B"); //select console 2
        bluetooth.println("Stop"); //print stop to console 2
        bluetooth.print("*"); //end of write to console 1
        go=0; //set go to 0 to stop the program
        }
          if (btn==0x58) //if char X is received
          {
          bluetooth.print("*B"); //select console 2
          bluetooth.println("Clear"); //print clear to console 2
          bluetooth.print("*GC*"); //clear graph
          }
   } // else nothing
  }

/* ********************************** Bluetooth end *********************************** */

/*===================================== functions ======================================*/  
