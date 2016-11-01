  
  int leftS = 4;          //blå PD4 -   B00010000
  int centerLeftS = 7;    //lilla PD7 - B10000000
  int center = 8;         //hvid PB0 -  B00000001
  int centerRightS = 13;  //grøn PB5 -  B00100000
  int rightS = 2;         //brun PD2 -  B00000100
  float EPchangeL = 0;
  float EPchangeR = 0;
  void ReadSensors();
 
void setup() 
  {
    
  }

void loop() 
  {
    ReadSensors();
    delay(100);
  }

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
    //int val = 0;

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
        EPchangeL = 30;
        EPchangeR = -2;
      break;
  
      case B10000000: // OXOOO
        EPchangeL = 35;
        EPchangeR = -4;
      break;
  
      case B10010000: // XXOOO
        EPchangeL = 40;
        EPchangeR = -6;
      break;
    
      case B00010000: // XOOOO
        EPchangeL = 45;
        EPchangeR = -8;
      break;   
    
      case B00100001: // OOXXO
        EPchangeL = -2;
        EPchangeR = 30;
      break;    
    
      case B00100000: // OOOXO
        EPchangeL = -4;
        EPchangeR = 35;
      break;
    
      case B00100100: // OOOXX
        EPchangeL = -6;
        EPchangeR = 40;
      break;
  
      case B00000100: // OOOOX
        EPchangeL = -8;
        EPchangeR = 45;
      break;

      case B00000000:  // OOOOO
        if(EPchangeL == 45)
          {
            EPchangeL = 50;
            EPchangeR = -10;
          }
        else if(EPchangeR == 45)
          {
            EPchangeL = -10;
            EPchangeR = 50;
          }
      break;
    } 
  }
