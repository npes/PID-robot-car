
//pin definitions
  const int leftTaco=A4;
  const int rightTaco=A5;

//global 
int stateLeftSensor;
int stateRightSensor;

void setup() {

//  Serial.begin(9600);

  pinMode(rightTaco, INPUT);
  pinMode(leftTaco, INPUT);

  Serial.begin(9600);
}

void loop() {

//read the tacos
stateLeftSensor+=digitalRead (leftTaco);
//stateRightSensor+=digitalRead (rightTaco);
Serial.println(stateLeftSensor);
//Serial.println(stateRightSensor);
}
