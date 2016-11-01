#include <stdio.h>

/*This program drives the wheels of the robot forward and uses the tacometers to count the rotations in ticks
The ticks is summed in the variables int stateLeftTaco and int stateRightTaco. Speed is controlled with the variables slowL 
and slowR*/

/* ======================================= Pins setup =========================================== */

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
//sum taco counts
  int countleftTacho=0;
  int countrightTacho=0;

//tacho state checks
  int stateleftTacho=0;
  int leftlastTachostate=0;
  int staterightTacho=0;
  int rightlastTachostate=0;

//variables for PID functions
  const float Kp_value = 3.0;
  const float Ki_value = 0.3;
  const float Kd_value = 0.0003;
  
  short int old_DutyL = 80; // starting pwm for both motors
  short int old_DutyR = 80; // starting pwm for both motors
  
  short int NewDUTYL = 0;    // new pwm after PID
  short int NewDUTYR = 0;    // new pwm after PID 
  
  int setPointL = 8;         // Setpoint for both motors, number of freadings desired from pid call to pid call.
  int setPointR = 8;
  
  int PID_TacoL = 0;
  int PID_TacoR = 0;
  
  int PIDFlag = 0; // set when timer interrupt happens, and enables PID function to be called
  
  int LTC = 0; // used for serial printing values
  int RTC = 0; // used for serial printing values

  int SPchangeL = 0;
  int SPchangeR = 0;

/* ==================================== global variables End =========================================*/ 
  
// ======================================= Function prototypes =======================================
  int tachL(); // function to check left taco and eventually inc. taco counter
  int tachR(); // function to check right taco and eventually inc. taco counter

  int PIDFunctionL(int DutyL, int SetPointL); // PID function for motor control. requires pwm input as starting point
  int PIDFunctionR(int DutyR, int SetPointR); // PID function for motor control. requires pwm input as starting point
// ===================================== Function prototypes End =====================================

  
void setup() {
// enabling timer 1 for timer interrupt each 0,125 sek
  /*
  cli();//stop interrupts
  
  // set timer1 interrupt at 2Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 7812 ;// = (16*10^6) / (4*1024) - 1 (must be <65536) 
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  */
  
// =============================== set timer1 interrupt at 8Hz ===================================
  cli();//stop interrupts
  
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1952 ;// = (16*10^6) / (8*1024) - 1 (must be <65536) 
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts


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

// ====================================== moter speed settings ======================================
  analogWrite(motorleftSSpeed, old_DutyL);
  digitalWrite(motorleftSA, 1);
  digitalWrite(motorleftSB, 0);
   
  analogWrite(motorrightSSpeed, old_DutyR);
  digitalWrite(motorrightSA, 1);
  digitalWrite(motorrightSB, 0);

// ==================================== moter speed settings End =====================================  
}

void loop() {
  // check tacho functions and store returned value, for use in PID function
  LTC = tachL(); // function for left tacho
  RTC = tachR(); // function for right tacho

  int SetL = setPointL + SPchangeL;
  int SetR = setPointR + SPchangeR;
  
  // =======================================================================
    
  // When timer interrupt is executed a flag counter "PIDFlag" is set 1
  if(PIDFlag == 1){
    // calls PID functions for both motors and store returned result for use next time functuion is called
    old_DutyL = PIDFunctionL(old_DutyL , SetL);
    old_DutyR = PIDFunctionR(old_DutyR , SetR);
    
    Serial.print(SetL);
    Serial.print(" ");

    //Serial.println(SetR);
    //Serial.print(" ");
    PIDFlag = 0; // clear PIDFlag for next interrupt
   }
}

/*===================================== interrupts ======================================*/

// interrupt rotine for timer 1
ISR(TIMER1_COMPA_vect){
  // stores tacho values for later use in PID
  PID_TacoL = countleftTacho;
  PID_TacoR = countrightTacho;
  
  // Clears counters
  countleftTacho = 0;
  countrightTacho = 0;
  // sets flag for PID call i main code
  PIDFlag = 1;
}

/*===================================== interrupts ======================================*/

/*===================================== functions ======================================*/
/* ************************************ taco functions ******************************** */
  // check left tacho
  int tachL(){
    stateleftTacho = digitalRead(leftTacho); //read the left tacho
    if(stateleftTacho != leftlastTachostate){ //compare the tachostate to the last tachostate, if change enter the loop
      if(leftlastTachostate==0){
        leftlastTachostate=1;
      }else{
        leftlastTachostate=0;
      }
      if(stateleftTacho==HIGH)  { //if the state is high the Tacho went from low to high
        countleftTacho++; //increment left tacho counter
        return countleftTacho;
      }     
    }
  }

  // check right tacho
  int tachR(){
    staterightTacho=digitalRead (rightTacho); //read the right tacho
    if(staterightTacho != rightlastTachostate) { //compare the tachostate to the last tachostate, if change enter the loop
      if(rightlastTachostate==0) {
          rightlastTachostate=1;
      }else{
        rightlastTachostate=0;
      }
      if (staterightTacho==HIGH)  { //if the state is high the Tacho went from low to high
        countrightTacho++; //increment left tacho counter
        return countrightTacho;
      }  
    }
  }

/* ************************************ taco functions ******************************** */

/* ************************************ PID functions ********************************* */
  // ================================== PID function L ================================
  int PIDFunctionL(int DutyL, int SetPointL){

    short int ErrorTL;    // error value taco left
    float PTL;        // Holds the calculated Proportional value for taco left
    float ITL;        // integral for left taco
    short int I_errorTL;  // Holds the calculated Integral value for left taco
    float DTL;        // Holds the calculated Differential value for left taco
    short int D_errorTL;  // Holds the derivative error for left taco
    int SetPoint = SetPointL; 
    short int old_DutyL = DutyL; // store function input (pwm) for use in function
/*
    Serial.print("in: ");
    Serial.print(old_DutyL);
    Serial.print(" ");
*/    
    // Error of Set point vs FB for left taco
    ErrorTL = (SetPoint - PID_TacoL); // setpoint is wished value, FB is output from taco
    
    Serial.print(PID_TacoL);
    Serial.print(" ");
    
    // P factor and error
    PTL = Kp_value * ErrorTL;
    
    // I_erro over time factor of the I term
    I_errorTL += ErrorTL;
  
    // Set up a Integral windup protection. Set a max and a min level for the I error
  
    // I factor over time. Remember I is error over time. Which means if error is 1 over two iteration error is now 2
    ITL = Ki_value * I_errorTL;
  
    // D factor of the PID
    DTL = Kd_value * (D_errorTL - ErrorTL);
    
    // The New error becomes the error the D expects and if not the same it will adjust for it.
    D_errorTL = ErrorTL;
  
    // the PID SUM Logic
    NewDUTYL = old_DutyL + (PTL + ITL + DTL);

    //load the new duty cycle to the variable of the old so you can adjust for it.
    old_DutyL = NewDUTYL;

   // You can also implement a min max for your PWM value before you send it further.
   
   if(old_DutyL > 255)
    {
      old_DutyL = 255;
    }
    else if(old_DutyL < 60)
    {
       old_DutyL = 60;
    } 

       
    analogWrite(motorleftSSpeed, NewDUTYL);
    return old_DutyL;
    
  }
  // ============================== PID function L End ================================

// PID function R
  int PIDFunctionR(int DutyR, int SetPointR){
     
    short int ErrorTR;    // error value taco right
    float PTR;        // Holds the calculated Proportional value for taco right
    float ITR;        // integral for right taco
    short int I_errorTR;  // Holds the calculated Integral value for right taco
    float DTR;        // Holds the calculated Differential value for right taco
    short int D_errorTR;  // Holds the derivative error for right taco
    int SetPoint = SetPointR; 
    short int old_DutyR = DutyR; // store function input (pwm) for use in function
     
    // Error of Set point vs FB for right taco
    ErrorTR = (SetPoint - PID_TacoR); // setpoint is wished value, FB is output from taco

    Serial.println(PID_TacoR);
        
    // P factor and error
    PTR = Kp_value * ErrorTR;
    
    // I_erro over time factor of the I term
    I_errorTR += ErrorTR;
  
    // Set up a Integral windup protection. Set a max and a min level for the I error
  
    // I factor over time. Remember I is error over time. Which means if error is 1 over two iteration error is now 2
    ITR = Ki_value * I_errorTR;
  
    // D factor of the PID
    DTR = Kd_value * (D_errorTR - ErrorTR);
    // The New error becomes the error the D expects and if not the same it will adjust for it.
    D_errorTR = ErrorTR;
  
    // the PID SUM Logic
    NewDUTYR = old_DutyR + (PTR + ITR + DTR);
    
    //load the new duty cycle to the variable of the old so you can adjust for it.
    old_DutyR = NewDUTYR;

    if(old_DutyR > 255)
    {
      old_DutyR = 255;
    }
    else if(old_DutyR < 60)
    {
       old_DutyR = 60;
    }
    
    // You can also implement a min max for your PWM value before you send it further.
    analogWrite(motorrightSSpeed, NewDUTYR);
    return old_DutyR;
    
}
/* ************************************ PID functions ********************************* */
/*===================================== functions ======================================*/  
