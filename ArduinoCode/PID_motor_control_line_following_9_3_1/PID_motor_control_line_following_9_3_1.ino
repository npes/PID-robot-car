#include <stdio.h>
#include <SoftwareSerial.h>  

/*This program drives the wheels of the robot forward and uses the tacometers to count the rotations in ticks
The ticks is summed in the variables int stateLeftTaco and int stateRightTaco. Speed is controlled with the variables slowL 
and slowR*/

/* ======================================= Pins setup =========================================== */
//Bluetooth  
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
  int motorleftSA=11;     // pin 11 på arduino forbundet til pin 2 på motor driver
  int motorrightSA=10;    // pin 10 på arduino forbundet til pin 10 på motor driver
  int motorleftSB=9;      // pin 9 på arduino forbundet til pin 7 på motor driver
  int motorrightSB=3;     // pin 3 på arduino forbundet til pin 15 på motor driver
  int motorleftSSpeed=5;  // pin 5 på arduino forbundet til pin 1 på motor driver
  int motorrightSSpeed=6; // pin 6 på arduino forbundet til pin 9 på motor driver

/* ====================================== global variables ==========================================*/ 
// PID function
  const float Kp_value = 1; // 1: 2.5 ; 2: 4 ; 3: 5 ; 4:
  const float Ki_value = 0.001;
  const float Kd_value = 0.01; // 1: (2.35 - 2.5) ; 2: (3.9 - 4) ; 3: 4.9
  
// starting pwm
  short int old_DutyL = 70; 
  short int old_DutyR = 70; 

// setpoint in Hz
  int EXT_SetPoint = 45; // standard 40
   
// error value
  float EPchangeL; 
  float EPchangeR;

// Freq. for each tacho
  float TachoLeftFreq = 0;
  float TachoRightFreq = 0;
  
//start and stop from bluetooth
  int go=1; //used to determine start or stop from bluetooth buttons. 1=start 0=stop
  
/* ======================================= Function prototypes ======================================= */
// function to check tachos, the time periode / frequencies, used for checking error 
  void ReadTacho(); 

// Read front sensors/line  
  void ReadSensors();   
  
// PID Motor control. requires pwm input as starting point
  int PIDFunctionL(int PWMSignal, int InputSetPoint); 
  int PIDFunctionR(int PWMSignal, int InputSetPoint);

// bluetooth 
  int func_BT_init (void); //initializes bluetooth, call it in setup
  int func_BTGraph(int HertzL, int HertzR, int SetPointL); //prints 4 variables to a BlueTooth graph
  int func_BTPIDLValues(float P, float I, float D); //Prints PID variables to BlueTooth console 
  int func_BTPIDRValues(float P, float I, float D); //Prints PID variables to BlueTooth console 
  int func_BTreadbuttons (void); //Reads BlueTooth buttons when called

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

/* ====================================== Communication init ====================================== */
//Start serial communication
  Serial.begin(9600);

//initialize bluetooth and clear graph  
  func_BT_init(); 

/* ====================================== motor direction ==================================== */
// forward dirction for left motor 
  digitalWrite(motorleftSA, 1);
  digitalWrite(motorleftSB, 0);

// forward direction for right motor
  digitalWrite(motorrightSA, 1);
  digitalWrite(motorrightSB, 0);
}

void loop() 
{
     
  //Read BlueTooth buttons  
  func_BTreadbuttons (); 
  
  if (go==1) //start has been pressed
  {
  // drive forward, using PWM
    analogWrite(motorleftSSpeed, old_DutyL);  // use PWM values for left motor, initial PWM and after changed using PID 
    analogWrite(motorrightSSpeed, old_DutyR); // use PWM values for right motor, initial PWM and after changed using PID

  // Read values from tacho, store freq. value for both in a variable for each
    ReadTacho();   
    
    Serial.print(EXT_SetPoint);
    Serial.print(" ");
  // calls PID functions for both motors and store returned result for use next time functuion is called    
    old_DutyL = PIDFunctionL(old_DutyL , EXT_SetPoint); // PID_value er error fra line sensor
    old_DutyR = PIDFunctionR(old_DutyR , EXT_SetPoint); // PID_value er error fra line sensor
    
  // write Hertz and setpoint values to bluetooth graph   
     func_BTGraph(TachoLeftFreq, TachoRightFreq, EXT_SetPoint+EPchangeR); 
    
  }
  else //Go=0, start has not been pressed
  {
    analogWrite(motorleftSSpeed, 0);
    analogWrite(motorrightSSpeed, 0);
  }
}

/* ================================== PID function L =============================== */
  int PIDFunctionL(int PWMSignal, int InputSetPoint){
    
    short int Error;    // error value taco left
    float PL;           // Holds the calculated Proportional value for taco left
    float IL;           // integral for left taco
    short int I_error;  // Holds the calculated Integral value for left taco
    float DL;           // Holds the calculated Differential value for left taco
    short int D_error;  // Holds the derivative error for left taco
    ReadSensors();      // Read sensors to follow line, and change error for PID accordingly to the line
    int SetPoint = InputSetPoint+EPchangeL; // SetPoint as combination of original setPoint and line detection
    //Serial.print(SetPoint);
    //Serial.print(" ");
    short int old_PWM = PWMSignal; // store function input (pwm) for use in function
    short int New_PWM = 0;    
 
    
    // Error of Set point vs FB for left taco
    Error = (SetPoint - TachoLeftFreq);  // setpoint is wished value, TachoLeftFreq is output from taco
    if(Error == 0) // error equal to 0 means that the desired PWM value is reached. As a result is the cumulative error reset, to prevent a wind up effect 
    {
      I_error = 0;
    }
     
    // P factor and error
    PL = Kp_value * Error;
    
    // I_error over time factor of the I term
    I_error += Error;
  
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
    
    // Implement a min/max for your PWM value before you send it further.   
    constrain(old_PWM,0,255);
      
    analogWrite(motorleftSSpeed, old_PWM);
    func_BTPIDLValues(PL,IL,DL); //write P,I and D values to bluetooth console
    return old_PWM;
    
  }

/* ================================== PID function R =============================== */
  int PIDFunctionR(int PWMSignal, int InputSetPoint){
    
    short int Error;    // error value taco right
    float PR;           // Holds the calculated Proportional value for taco right
    float IR;           // integral for right taco
    short int I_error;  // Holds the calculated Integral value for right taco
    float DR;           // Holds the calculated Differential value for right taco
    short int D_error;  // Holds the derivative error for right taco
    ReadSensors();      // line detection
    int SetPoint = InputSetPoint + EPchangeR; // Setpoint based on line detection 
    //Serial.print(SetPoint);
    //Serial.println(" ");
    short int old_PWM = PWMSignal; // store function input (pwm) for use in function
    short int New_PWM = 0;
    
    // Error of Set point vs FB for right taco
    Error = (SetPoint - TachoRightFreq); // setpoint is wished value, FB is output from taco
    if(Error == 0) // error equal to 0 means that the desired PWM value is reached. As a result is the cumulative error reset, to prevent a wind up effect 
    {
      I_error = 0;
    }
    
    // P factor and error
    PR = Kp_value * Error;
    
    // I_error over time factor of the I term
    I_error += Error;
  
    // I factor over time. Remember I is error over time. Which means if error is 1 over two iteration error is now 2
    IR = Ki_value * I_error;
  
    // D factor of the PID
    DR = Kd_value * (D_error - Error);
    
    // The New error becomes the error the D expects and if not the same it will adjust for it.
    D_error = Error;
  
    // the PID SUM Logic
    New_PWM = old_PWM + (PR + IR + DR);
    
    // load the new duty cycle to the variable of the old so you can adjust for it.
    old_PWM = New_PWM;
    
    // Implement a min/max for your PWM value before you send it further.   
    constrain(old_PWM,0,255); 
        
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
    Serial.print(TachoRightFreq);
    Serial.print(" ");
 
  }
  
/* ********************************* Tachometer readings ****************************** */

/* ********************************* Sensor read start ******************************** */
  void ReadSensors()
  {
    
/*
 * X0000 = B00010000
 * XX000 = B10010000
 * 0X000 = B10000000
 * 0XX00 = B10000001
 * 00X00 = B00000001
 * 00XX0 = B00100001
 * 000X0 = B00100000
 * 000XX = B00100100
 * 0000X = B00000100
 * 00000 = B00000000
 */
    
    // Sensor state - read sensors and store in var
    int SenseB=PINB & B00100001; //read port B sensor pins
    int SenseD=PIND & B10010100; //read port D sensor pins
    int val=SenseB|SenseD; 
 
  switch (val)
    {
      case B00000001: // OOXOO
        EPchangeR = 0;
        EPchangeL = 0;
      break;
      
      case B10000001: // OXXOO
        EPchangeL = 3;
        EPchangeR = 0;
      break;
      
      case B10000000: // OXOOO
        EPchangeL = 4;
        EPchangeR = 0;
      break;
      
      case B10010000: // XXOOO
        EPchangeL = 6;
        EPchangeR = 0;
      break;
      
      case B00010000: // XOOOO
        EPchangeL = 7;
        EPchangeR = 0;
      break;   
      
      case B00100001: // OOXXO
        EPchangeL = 0;
        EPchangeR = 3;
      break;    
      
      case B00100000: // OOOXO
        EPchangeL = 0;
        EPchangeR = 4;
      break;
      
      case B00100100: // OOOXX
        EPchangeL = 0;
        EPchangeR = 6;
      break;
      
      case B00000100: // OOOOX
        EPchangeL = 0;
        EPchangeR = 7;
      break;

      /*case B00000000:  // OOOOO
        if(EPchangeL == 12)
          {
            EPchangeL = 12;
            EPchangeR = -4;
          }
        else if(EPchangeR == 12)
          {
            EPchangeL = -4;
            EPchangeR = 12;
          }
      break;
      */
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

int func_BTGraph(int HertzL, int HertzR, int SetPointL) //prints 3 variables to a BlueTooth graph
  {
    bluetooth.print("*G"); //select graph
    bluetooth.print(HertzL); //print HertzL to trace 1 
    bluetooth.print(".1"); 
    bluetooth.print(",");
    bluetooth.print(HertzR); //print HertzR to trace 2 
    bluetooth.print(".2"); 
    bluetooth.print(",");
    bluetooth.print(SetPointL); //print SetPoint to trace 3 
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
