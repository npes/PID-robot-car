/*
 * Line-Following Robot with PID
 * 
 * Erhvervsakademiet Lillebaelt
 * 
 * Credits:
 * - Daniel Larsen
 * - Martin Salling
 * - Jonas Thomsen
 */
 
//contractor so you can call a function that runs after the Main

#define SOFTTURN 22250
#define HARDTURN 22000 

// Function prototypes
void lMotor(int spd, bool Forth);
void rMotor(int spd, bool Forth);
void bMotors(int spd, bool Forth);
void rotate(bool anticlockwise, int Time);
int readTahoL(int Leftcount);
int readTahoR(int Rightcount);
int PIDFunctionLeft(int PWMLeft);
int PIDFunctionRight(int PWMRight);

// Pin numbers
const int lSensor = 4; // Left sensor
const int mSensor = 8; // mid sensor
const int rSensor = 2; // Right sensor
const int lMotorPinForth = 11; // Left motor forth (when HIGH and opposite of lMotorPinBack)
const int lMotorPinBack = 9; // Left motor back (when HIGH and opposite of lMotorPinForth)
const int rMotorPinForth = 10; // Right motor (when HIGH and opposite of rMotorPinBack)
const int rMotorPinBack = 3; // Right motor back (when HIGH and opposite of rMotorPinForth)
const int tacoSensorLeft = A4; // Left tacosensor
const int tacoSensorRight = A5; // Right tacosensor
const int EnablePinLeft = 5;
const int EnablePinRight = 6;

//Varibles
const bool hvid = 0;
const bool sort = 1;
float NewDUTYLeft;
float NewDUTYRight;
float PWMLeft = 100; // Preset PWM to start out with on left motor
float PWMRight = 100; // Preset PWM to start out with on right motor
int TacoValLeft;
int TacoValRight;
int timesleft;
int timesright;
bool right, mid, left;
unsigned long DurationLeftHigh, DurationRightHigh, DurationLeftLow, DurationRightLow, TotalDurationLeft, TotalDurationRight;

void setup() 
{
  Serial.begin(250000);
  pinMode(lMotorPinForth, OUTPUT);
  pinMode(lMotorPinBack, OUTPUT);
  pinMode(rMotorPinForth, OUTPUT);
  pinMode(rMotorPinBack, OUTPUT);
  pinMode (EnablePinLeft, OUTPUT);
  pinMode (EnablePinRight, OUTPUT);
  pinMode(tacoSensorLeft, INPUT);
  pinMode(tacoSensorRight, INPUT);
  pinMode(lSensor, INPUT);
  pinMode(mSensor, INPUT);
  pinMode(rSensor, INPUT);
  digitalWrite(rMotorPinBack, LOW);
  digitalWrite(lMotorPinBack, LOW);  
  digitalWrite(EnablePinLeft, HIGH);
  digitalWrite(EnablePinRight, HIGH);

}
   
void loop() 
{
  DurationLeftHigh = pulseIn(tacoSensorLeft, HIGH); // Take the time of a high pulse 
  DurationLeftLow = pulseIn(tacoSensorLeft, LOW); // Take the time of a low pulse
  DurationRightHigh = pulseIn(tacoSensorRight, HIGH);
  DurationRightLow = pulseIn(tacoSensorRight, LOW);

  TotalDurationLeft = DurationLeftHigh + DurationLeftLow; // Sum up the high and low pulse to get the full duty cycle
  TotalDurationRight = DurationRightHigh + DurationRightLow;
  
  right = digitalRead(rSensor);
  mid = digitalRead(mSensor);
  left = digitalRead(lSensor);
  PWMLeft = PIDFunctionLeft(PWMLeft);
  PWMRight = PIDFunctionRight(PWMRight);
}

int PIDFunctionLeft(int PWMLeft)
{
    float Kp_value = 0.008;
    float Ki_value = 0.00008;
    float Kd_value = 0;

    long Error; //error value
    float P;          // Holds the calculated Proportional value
    float I;
    float I_error;    // Holds the calculated Integral value
    float D;          // Holds the calculated Differential value
    float D_error;
    int SetPoint = 22500;
    unsigned int old_Duty = PWMLeft;

    if(left==0 && mid==1 && right==1) SetPoint = SOFTTURN;
    else if(left==0 && mid==0 && right==1) SetPoint = HARDTURN;
    
    //Serial.print(left);
    //Serial.print(mid);
    //Serial.println(right);
    
    //Serial.print("TotalDurationLeft = ");
    //Serial.println(TotalDurationLeft);
    //  Error of Set point vs FB
    Error = (SetPoint - TotalDurationLeft);
    //Serial.print("Error = ");
    //Serial.println(Error);
    // P factor and error
    P = Kp_value * Error;
    //Serial.print("p = ");
    //Serial.println(P);
    // I_erro over time factor of the I term
    I_error += Error;
    //Serial.print("I_error = ");
    //Serial.println(I_error);
    // Set up a Integral windup protection. Set a max and a min level for the I error

    // I factor over time. Remember I is error over time. Which means if error is 1 over two iteration error is now 2
    I = Ki_value * I_error;
    //Serial.print("I = ");
    //Serial.println(I);
    // D factor of the PID
    D = Kd_value * (D_error - Error);
    //Serial.print("D = ");
    //Serial.println(D);
    // The New error becomes the error the D expects and if not the same it will adjust for it.
    D_error = Error;

    // the PID SUM Logic
    NewDUTYLeft = old_Duty - (P + I +D);
    //load the new duty cycle to the variable of the old so you can adjust for it.
    if(NewDUTYLeft>=256)      // If the PWM is above 256 it will be set to 255 PWM
    {
      NewDUTYLeft = 255;
    }

    else if(NewDUTYLeft<=-69)  // If the PWM is below 69 it will be set to 70 PWM 
    {
      NewDUTYLeft = 70;
    }

    //Serial.print("NewDUTYLeft = ");
    //Serial.print("NewDUTYLeft: ");
    //Serial.print(NewDUTYLeft);
    //Serial.print("   ");   
    
    old_Duty = NewDUTYLeft;
    //Serial.print("old_Duty = ");
    //Serial.println(old_Duty);    
    //Serial.print(" ");
    //Serial.print(" ");
     // You can also implement a min max for your PWM value before you send it further.
    
     analogWrite(lMotorPinForth, old_Duty); // Set left motor to the new duty cycle

     return old_Duty;
}

int PIDFunctionRight(int PWMRight)
{
    float Kp_value = 0.008;
    float Ki_value = 0.0008;
    float Kd_value = 0;

    long Error; //error value
    float P;      // Holds the calculated Proportional value
    float I;
    float I_error;    // Holds the calculated Integral value
    float D;    // Holds the calculated Differential value
    float D_error;
    int SetPoint = 22500;
    unsigned int old_Duty = PWMRight;
   
    //Serial.print("TotalDurationRight = ");
    //Serial.println(TotalDurationRight);

    if(left==1 && mid==1 && right==0) SetPoint = SOFTTURN;
    else if(left==1 && mid==0 && right==0) SetPoint = HARDTURN;

    //Serial.print(left);
    //Serial.print(mid);
    //Serial.println(right);
    
    // Error of Set point vs FB
    Error = (SetPoint - TotalDurationRight);
    //Serial.print(" ");
    //Serial.println(Error);
    // P factor and error
    P = Kp_value * Error;
    //Serial.print("p = ");
    //Serial.println(P);
    // I_erro over time factor of the I term
    I_error += Error;
    //Serial.print("I_error = ");
    //Serial.println(I_error);
    // Set up a Integral windup protection. Set a max and a min level for the I error

    // I factor over time. Remember I is error over time. Which means if error is 1 over two iteration error is now 2
    I = Ki_value * I_error;
    //Serial.print("I = ");
    //Serial.println(I);
    // D factor of the PID
    D = Kd_value * (D_error - Error);
    // The New error becomes the error the D expects and if not the same it will adjust for it.
    D_error = Error;
    //Serial.print("D = ");
    //Serial.println(D);
    // the PID SUM Logic
    NewDUTYRight = old_Duty - (P + I +D);           
    //load the new duty cycle to the variable of the old so you can adjust for it.
    if(NewDUTYRight>=256)     // If the PWM is above 256 it will be set to 255 PWM
    {
      NewDUTYRight = 255;
    }

    else if(NewDUTYRight<=69) // If the PWM is below 69 it will be set to 70 PWM 
    {
      NewDUTYRight = 70;
    }
    //Serial.print("NewDUTYRight ");
    //Serial.print(NewDUTYRight);
    //Serial.print("\r");
    
    old_Duty = NewDUTYRight;
     // You can also implement a min max for your PWM value before you send it further.
     analogWrite(rMotorPinForth, old_Duty); // Set right motor to the new duty cycle
     return old_Duty;
}

