#include <stdio.h>

/*This program drives the wheels of the robot forward and uses the tacometers to count the rotations in ticks
The ticks is summed in the variables int stateLeftTaco and int stateRightTaco. Speed is controlled with the variables slowL 
and slowR*/

/*Pins setup*/

//Taco meters
  const int leftTacho=A4;
  const int rightTacho=A5;

// sensors
//  int center = 8; //hvid
//  int centerLeftS = 7; //lilla
//  int centerRightS = 13; //grøn
//  int leftS = 4; //blå
//  int rightS = 2; //brun

// motor pins
  int motorleftSA=11; // pin 11 på arduino forbundet til pin 2 på motor driver
  int motorrightSA=10; // pin 10 på arduino forbundet til pin 10 på motor driver
  int motorleftSB=9; // pin 9 på arduino forbundet til pin 7 på motor driver
  int motorrightSB=3; // pin 3 på arduino forbundet til pin 15 på motor driver
  int motorleftSSpeed=5; // pin 5 på arduino forbundet til pin 1 på motor driver
  int motorrightSSpeed=6; // pin 6 på arduino forbundet til pin 9 på motor driver

/*global variables*/ 
//sum taco counts
  int countleftTacho=0;
  int countrightTacho=0;

//tacho state checks
  int stateleftTacho=0;
  int leftlastTachostate=0;
  int staterightTacho=0;
  int rightlastTachostate=0;
  
//Function prototypes
  int tachL(); // function to check left taco and eventually inc. taco counter
  int tachR(); // function to check right taco and eventually inc. taco counter
  int PIDFunction(short int Duty); // PID function for motor control. requires pwm input as starting point


//Variables
  short int old_Duty = 200; // starting pwm for both motors
  short int NewDUTY = 0; // new pwm after PID 
  int PID_TacoL = 0;
  int PID_TacoR = 0;
  int PIDFlag = 0;
  int pwmValue = 0;
  int LTC = 0;
  int RTC = 0;
  
void setup() {
// enabeling timer 1 for timer interrupt each 0,5 sek
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

  sei();//allow interrupts


/*pin direction setup*/
  // sensors
/*  
  pinMode (center, INPUT);
  pinMode (centerLeftS, INPUT);
  pinMode (centerRightS, INPUT);
  pinMode (leftS, INPUT);
  pinMode (rightS, INPUT);
*/  

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

  Serial.begin(115200);

  // moter speed settings
  analogWrite(motorleftSSpeed, old_Duty);
  digitalWrite(motorleftSA, 1);
  digitalWrite(motorleftSB, 0);
   
  analogWrite(motorrightSSpeed, old_Duty);
  digitalWrite(motorrightSA, 1);
  digitalWrite(motorrightSB, 0);
}

void loop() {
  // check left tacho
   LTC = tachL();
  // check right tacho
  RTC = tachR();
  if(PIDFlag == 1){
    old_Duty = PIDFunction(old_Duty);
    Serial.print("L: ");
    Serial.print(LTC);
    Serial.print(" ");
    Serial.print("R: ");
    Serial.print(RTC);
    Serial.println( );
  }
}

/*===================================== interrupts ======================================*/

// interrupt rotine for timer 1 after app. 0,5 sek
ISR(TIMER1_COMPA_vect){
  PID_TacoL = countleftTacho;
  PID_TacoR = countrightTacho;
  countleftTacho = 0;
  countrightTacho = 0;
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
  // PID function
  int PIDFunction(short int Duty){
    PIDFlag = 0; // clear PIDFlag for next interrupt
    const float Kp_value = 0.1;
    const float Ki_value = 0.1;
    const float Kd_value = 0.0001;
  
    short int ErrorTL;    // error value taco left
    float PTL;        // Holds the calculated Proportional value for taco left
    float ITL;        // integral for left taco
    short int I_errorTL;  // Holds the calculated Integral value for left taco
    float DTL;        // Holds the calculated Differential value for left taco
    short int D_errorTL;  // Holds the derivative error for left taco
    short const int SetPoint = 20; 
    short int old_Duty = Duty; // store function input (pwm) for use in function
    
    
    // Error of Set point vs FB for left taco
    ErrorTL = (SetPoint - PID_TacoL); // setpoint is wished value, FB is output from taco
    
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
    NewDUTY = old_Duty + (PTL + ITL + DTL);
  /*
    Serial.print("Error: ");
    Serial.println(ErrorTL);
    Serial.print("old: ");
    Serial.println(old_Duty);
    
    Serial.print("new: ");
    Serial.println(NewDUTY);
    */
    //load the new duty cycle to the variable of the old so you can adjust for it.
    old_Duty = NewDUTY;

     // You can also implement a min max for your PWM value before you send it further.
     
     analogWrite(motorleftSSpeed, NewDUTY);
     return old_Duty;
    
}
/* ************************************ PID functions ********************************* */
/*===================================== functions ======================================*/  
