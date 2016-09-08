/*This program drives the wheels of the robot forward and uses the tacometers to count the rotations in ticks
The ticks is summed in the variables int stateLeftTaco and int stateRightTaco. Speed is controlled with the variables slowL 
and slowR*/

/*Pins setup*/

//Taco meters
  const int leftTacho=A4;
  const int rightTacho=A5;

// sensors
  int center = 8; //hvid
  int centerLeftS = 7; //lilla
  int centerRightS = 13; //grøn
  int leftS = 4; //blå
  int rightS = 2; //brun

// motor pins
  int motorleftSA=11; // pin 11 på arduino forbundet til pin 2 på motor driver
  int motorrightSA=10; // pin 10 på arduino forbundet til pin 10 på motor driver
  int motorleftSB=9; // pin 9 på arduino forbundet til pin 7 på motor driver
  int motorrightSB=3; // pin 3 på arduino forbundet til pin 15 på motor driver
  int motorleftSSpeed=5; // pin 5 på arduino forbundet til pin 1 på motor driver
  int motorrightSSpeed=6; // pin 6 på arduino forbundet til pin 9 på motor driver

// car light pins
  int frontLeft = A0;
  int frontRight = A1;
  int backLeft = A2;
  int backRight = A3;


/*global variables*/ 

//sum taco counts
  int countleftTacho=0;
  int countrightTacho=0;

//tacho state checks
  int stateleftTacho=0;
  int leftlastTachostate=0;
  int staterightTacho=0;
  int rightlastTachostate=0;
  
//PWM Speeds
  int slowL = 90;
  int slowR = 90;
  int middleL = 90;
  int middleR = 90;
  int fastL = 90;
  int fastR = 90;
  int offL = 0;
  int offR = 0;

/*Function prototypes*/
  int DriveForwardLeftWheel(int spd);
  int DriveForwardRightWheel(int spd);
  int DriveReverseLeftWheel(int spd);
  int DriveReverseRightWheel(int spd);
  int carLight(int FL, int FR, int BL, int BR);
  
void setup() {

/*pin direction setup*/
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

  Serial.begin(9600);
}

void loop() {

stateleftTacho=digitalRead (leftTacho); //read the left tacho
if (stateleftTacho != leftlastTachostate) { //compare the tachostate to the last tachostate, if change enter the loop
  if (stateleftTacho==HIGH)  { //if the state is high the Tacho went from low to high
    countleftTacho++; //increment left tacho counter
    Serial.println(countleftTacho); //Serial print the tacho value
    }
}
staterightTacho=digitalRead (rightTacho); //read the right tacho
if (staterightTacho != rightlastTachostate) { //compare the tachostate to the last tachostate, if change enter the loop
  if (staterightTacho==HIGH)  { //if the state is high the Tacho went from low to high
    countrightTacho++; //increment left tacho counter
    Serial.println(countrightTacho); //Serial print the tacho value
    }
}

// PID



//Decide how to drive based on sensors

// Read the sensors
int stateCenter = digitalRead (center);
int stateCenterLeftS = digitalRead (centerLeftS);
int stateCenterRightS = digitalRead (centerRightS);
int stateLeftS = digitalRead (leftS);
int stateRightS = digitalRead (rightS);

if (stateLeftS==0&&stateCenterLeftS==0&&stateCenter==0&&stateCenterRightS==0&&stateRightS==0) //1
    { //reverse
    DriveReverseLeftWheel(slowL);
    DriveReverseRightWheel(slowR);
    carLight(0, 0, 1, 1);
    delay(200);
    }
else if (stateLeftS==0&&stateCenterLeftS==0&&stateCenter==0&&stateCenterRightS==0&&stateRightS==1) //2
    { //right sharp
    DriveForwardLeftWheel(slowL); // middle to slow
    DriveReverseRightWheel(slowR);
    carLight(0, 1, 0, 1);
    }
else if (stateLeftS==0&&stateCenterLeftS==0&&stateCenter==0&&stateCenterRightS==1&&stateRightS==0) //3
    { //right
    DriveForwardLeftWheel(fastL);
    DriveForwardRightWheel(slowR);
    carLight(0, 1, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==0&&stateCenter==0&&stateCenterRightS==1&&stateRightS==1) //4
    { //right
    DriveForwardLeftWheel(fastL);
    DriveForwardRightWheel(slowR);
    carLight(0, 1, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==0&&stateCenter==1&&stateCenterRightS==0&&stateRightS==0) //5    
    { //straight
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    delay(50);
    }
else if (stateLeftS==0&&stateCenterLeftS==0&&stateCenter==1&&stateCenterRightS==0&&stateRightS==1) //6
    { //right
    DriveForwardLeftWheel(fastL);
    DriveForwardRightWheel(slowR);
    carLight(0, 1, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==0&&stateCenter==1&&stateCenterRightS==1&&stateRightS==0) //7
    { //right
    DriveForwardLeftWheel(fastL);
    DriveForwardRightWheel(slowR);
    carLight(0, 1, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==0&&stateCenter==1&&stateCenterRightS==1&&stateRightS==1) //8
    { //right sharp
    DriveForwardLeftWheel(slowL); 
    DriveReverseRightWheel(slowR);
    carLight(0, 1, 0, 1);
    }
else if (stateLeftS==0&&stateCenterLeftS==1&&stateCenter==0&&stateCenterRightS==0&&stateRightS==0) //9
    { //left
    DriveForwardLeftWheel(slowL);
    DriveForwardRightWheel(fastR);
    carLight(1, 0, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==1&&stateCenter==0&&stateCenterRightS==0&&stateRightS==1) //10
    { //left
    DriveForwardLeftWheel(slowL);
    DriveForwardRightWheel(fastR);
    carLight(1, 0, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==1&&stateCenter==0&&stateCenterRightS==1&&stateRightS==0) //11
    { //straight
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==1&&stateCenter==0&&stateCenterRightS==1&&stateRightS==1) //12
    { //straight
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==1&&stateCenter==1&&stateCenterRightS==0&&stateRightS==0) //13
    { //left
    DriveForwardLeftWheel(slowL);
    DriveForwardRightWheel(fastR);
    carLight(1, 0, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==1&&stateCenter==1&&stateCenterRightS==0&&stateRightS==1) //14
    { //straightR
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==1&&stateCenter==1&&stateCenterRightS==1&&stateRightS==0) //15
    { //straight
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    }
else if (stateLeftS==0&&stateCenterLeftS==1&&stateCenter==1&&stateCenterRightS==1&&stateRightS==1) //16
    { //right shaerp
    DriveForwardLeftWheel(slowL);
    DriveReverseRightWheel(slowR);
    carLight(0, 1, 0, 1);
    }
else if (stateLeftS==1&&stateCenterLeftS==1&&stateCenter==1&&stateCenterRightS==1&&stateRightS==1) //17
    { //straight
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==1&&stateCenter==1&&stateCenterRightS==1&&stateRightS==0) //18
    { //left sharp
    DriveReverseLeftWheel(slowL);
    DriveForwardRightWheel(slowR);
    carLight(1, 0, 1, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==1&&stateCenter==1&&stateCenterRightS==0&&stateRightS==1) //19
    { //left
    DriveForwardLeftWheel(slowL);
    DriveForwardRightWheel(fastR);
    carLight(1, 0, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==1&&stateCenter==1&&stateCenterRightS==0&&stateRightS==0) //20
    { //left sharp
    DriveReverseLeftWheel(slowL);
    DriveForwardRightWheel(slowR);
    carLight(1, 0, 1, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==1&&stateCenter==0&&stateCenterRightS==1&&stateRightS==1) //21
    { //straight
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==1&&stateCenter==0&&stateCenterRightS==1&&stateRightS==0) //22
    { //left
    DriveForwardLeftWheel(slowL);
    DriveForwardRightWheel(fastR);
    carLight(1, 0, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==1&&stateCenter==0&&stateCenterRightS==0&&stateRightS==1) //23
    { //left
    DriveForwardLeftWheel(slowL);
    DriveForwardRightWheel(fastR);
    carLight(1, 0, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==1&&stateCenter==0&&stateCenterRightS==0&&stateRightS==0) //24
    { //left
    DriveForwardLeftWheel(slowL);
    DriveForwardRightWheel(fastR);
    carLight(1, 0, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==0&&stateCenter==1&&stateCenterRightS==1&&stateRightS==1) //25
    { //right sharp
    DriveForwardLeftWheel(slowL); 
    DriveReverseRightWheel(slowR);
    carLight(0, 1, 0, 1);
    }
else if (stateLeftS==1&&stateCenterLeftS==0&&stateCenter==1&&stateCenterRightS==1&&stateRightS==0) //26
    { //right
    DriveForwardLeftWheel(fastL);
    DriveForwardRightWheel(slowR);
    carLight(0, 1, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==0&&stateCenter==1&&stateCenterRightS==0&&stateRightS==1) //27
    { //straight
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==0&&stateCenter==1&&stateCenterRightS==0&&stateRightS==0) //28
    { //left
    DriveForwardLeftWheel(slowL);
    DriveForwardRightWheel(fastR);
    carLight(1, 0, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==0&&stateCenter==0&&stateCenterRightS==1&&stateRightS==1) //29
    { //right sharp
    DriveForwardLeftWheel(slowL); 
    DriveReverseRightWheel(slowR);
    carLight(0, 1, 0, 1);
    }
else if (stateLeftS==1&&stateCenterLeftS==0&&stateCenter==0&&stateCenterRightS==1&&stateRightS==0) //30
    { //straight
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==0&&stateCenter==0&&stateCenterRightS==0&&stateRightS==1) //31
    { //straight
    DriveForwardLeftWheel(middleL);
    DriveForwardRightWheel(middleR);
    carLight(1, 1, 0, 0);
    }
else if (stateLeftS==1&&stateCenterLeftS==0&&stateCenter==0&&stateCenterRightS==0&&stateRightS==0) //32
    { //left sharp
    DriveReverseLeftWheel(slowL);
    DriveForwardRightWheel(middleR);
    carLight(1, 0, 1, 0);
    }
}


//**** functions ****//
  int DriveForwardLeftWheel (int spd) {
    int spdsetting = spd;
    analogWrite(motorleftSSpeed, spdsetting);
    digitalWrite(motorleftSA, 1);
    digitalWrite(motorleftSB, 0);
  }
  int DriveForwardRightWheel (int spd) {
    int spdsetting = spd;  
    digitalWrite(motorrightSA, 1);
    digitalWrite(motorrightSB, 0);
    analogWrite(motorrightSSpeed, spdsetting);    
  }
  int DriveReverseLeftWheel (int spd) {
    int spdsetting = spd;
    analogWrite(motorleftSSpeed, spdsetting);
    digitalWrite(motorleftSA, 0);
    digitalWrite(motorleftSB, 1);
  }
  int DriveReverseRightWheel (int spd) {
    int spdsetting = spd;
    digitalWrite(motorrightSA, 0);
    digitalWrite(motorrightSB, 1);
    analogWrite(motorrightSSpeed, spdsetting);    
  }

  int carLight(int FL, int FR, int BL, int BR){
    /*if(FL == true && FR == true && FL == true && FR == true){// all light up
      digitalWrite(frontLeft, 1);
      digitalWrite(frontRight, 1);
      digitalWrite(backLeft, 1);
      digitalWrite(backRight, 1);
    }else*/ 
    if(FL == true && FR == true && BL == false && BR == false){ // front light up
      digitalWrite(frontLeft, 1);
      digitalWrite(frontRight, 1);
      digitalWrite(backLeft, 0);
      digitalWrite(backRight, 0);
    }else if(FL == true && FR == false && BL == false && BR == false){ // left front light up
      digitalWrite(frontLeft, 1);
      digitalWrite(frontRight, 0);
      digitalWrite(backLeft, 0);
      digitalWrite(backRight, 0);
    }else if(FL == false && FR == true && BL == false && BR == false){ // right front ligth up
      digitalWrite(frontLeft, 0);
      digitalWrite(frontRight, 1);
      digitalWrite(backLeft, 0);
      digitalWrite(backRight, 0);
    }else if(FL == false && FR == false && BL == true && BR == false){ // back left light up 
      digitalWrite(frontLeft, 0);
      digitalWrite(frontRight, 0);
      digitalWrite(backLeft, 1);
      digitalWrite(backRight, 0);
    }else if(FL == false && FR == false && BL == false && BR == true){ // back right light up
      digitalWrite(frontLeft, 0);
      digitalWrite(frontRight, 0);
      digitalWrite(backLeft, 0);
      digitalWrite(backRight, 1);
    }else if(FL == false && FR == false && BL == true && BR == true){ // back light up
      digitalWrite(frontLeft, 0);
      digitalWrite(frontRight, 0);
      digitalWrite(backLeft, 1);
      digitalWrite(backRight, 1);
    }else if(FL == true && FR == false && BL == true && BR == false){ // light left side up
      digitalWrite(frontLeft, 1);
      digitalWrite(frontRight, 0);
      digitalWrite(backLeft, 1);
      digitalWrite(backRight, 0);
    }else if(FL == false && FR == true && BL == false && BR == true){ // light right side up
      digitalWrite(frontLeft, 0);
      digitalWrite(frontRight, 1);
      digitalWrite(backLeft, 0);
      digitalWrite(backRight, 1);
    }else if(FL == false && FR == true && BL == false && BR == true){ // light right side up
      digitalWrite(frontLeft, 0);
      digitalWrite(frontRight, 1);
      digitalWrite(backLeft, 0);
      digitalWrite(backRight, 1);
    }else if(FL == false && FR == false && BL == false && BR == false){ // lights out
      digitalWrite(frontLeft, 0);
      digitalWrite(frontRight, 0);
      digitalWrite(backLeft, 0);
      digitalWrite(backRight, 0);
    }
  }
