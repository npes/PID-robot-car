/*This program drives the wheels of the robot forward and uses the tacometers to count the rotations in ticks
The ticks is summed in the variables int stateLeftTaco and int stateRightTaco. Speed is controlled with the variables left and right*/

/*variables setup*/

//Taco meters
  const int leftTaco=A4;
  const int rightTaco=A5;
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
//global 
  //sum taco counts
  int stateLeftTaco;
  int stateRightTaco;
  //PWM Speeds
  int left = 110;
  int right = 110;
  
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
//Taco meters
  pinMode(rightTaco, INPUT);
  pinMode(leftTaco, INPUT);

  Serial.begin(9600);
}

void loop() {

//Drive the wheels forward slowly
//  DriveForwardLeftWheel(left);
//  DriveForwardRightWheel(right);

//read the tacos
stateLeftTaco+=digitalRead (leftTaco);
stateRightTaco+=digitalRead (rightTaco);
//Serial print the taco value
Serial.print(stateLeftTaco);
Serial.print("    ");
Serial.println(stateRightTaco);
delay (500);
}

//**** functions ****//
  int DriveForwardLeftWheel(int spd) {
    int spdsetting = spd;
    analogWrite(motorleftSSpeed, spdsetting);
    digitalWrite(motorleftSA, 1);
    digitalWrite(motorleftSB, 0);
  }
  int DriveForwardRightWheel(int spd) {
    int spdsetting = spd;  
    digitalWrite(motorrightSA, 1);
    digitalWrite(motorrightSB, 0);
    analogWrite(motorrightSSpeed, spdsetting);    
  }
  int DriveReverseLeftWheel(int spd) {
    int spdsetting = spd;
    analogWrite(motorleftSSpeed, spdsetting);
    digitalWrite(motorleftSA, 0);
    digitalWrite(motorleftSB, 1);
  }
  int DriveReverseRightWheel(int spd) {
    int spdsetting = spd;
    digitalWrite(motorrightSA, 0);
    digitalWrite(motorrightSB, 1);
    analogWrite(motorrightSSpeed, spdsetting);    
  }
