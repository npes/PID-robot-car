/*
Sketch to test bluetooth communication between PID robot car and http://keuwl.com/apps/bluetoothelectronics/
The sketch is using a custom panel with a graph to show the PID output and the P,I and D values as numbers.
The panel includes a start button to start the sketch, a stop button to stop it and a clear button to clear the graph.

Information about BT module taken from https://learn.sparkfun.com/tutorials/using-the-bluesmirf
 BTA=0006667D9784 (OUR bluetooth address)
 Get commands: D and E
 Set commands:  SN,<name> (set bluetooth device name)
                SF,1 (reset to default)
                ST,0 (config timer off)
 Action commands:
                I,<value> (search for other Bluetooth modules in range. value=search time)
*/
#include <SoftwareSerial.h>  
int bluetoothTx = A0;  // TX-O pin of bluetooth
int bluetoothRx = A1;  // RX-I pin of bluetooth
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

int go=0; //used to determine start or stop from bluetooth buttons. 1=start 0=stop

int func_BT_init (void); //initializes bluetooth, call it in setup
int func_BTGraph(int Hertz, int SetPoint); //prints 2 variables to a BlueTooth graph
int func_BTPIDValues(int P, float I, float D); //Prints PID variables to BlueTooth console 
int func_BTreadbuttons (void); //Reads BlueTooth buttons when called

void setup()
{
  func_BT_init();
  bluetooth.print("*GC*"); //Clear bluetooth graph  
}

void loop()
{
  func_BTreadbuttons (); Read BlueTooth buttons
  if (go==1) //start has been pressed
    {
    int count=random(-100,100); //store a random number in count
    int count2=0; //store 0 in count
    func_BTGraph(count, count2); //write count and count2 values to bluetooth graph
    int P=3; //store 3 in P
    float I=0.3; //store 0.3 in P
    float D=0.03; //store 0.03 in P
    func_BTPIDValues(P,I,D); //write P,I and D values to bluetooth console 
    delay(200); //wait 200 mS
    } // else nothing
}

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
  }

int func_BTGraph(int Hertz, int SetPoint) //prints 2 variables to a BlueTooth graph 
  {
    bluetooth.print("*G"); //select graph
    bluetooth.print(Hertz); //print Hertz to trace 1 
    bluetooth.print(".1"); 
    bluetooth.print(",");
    bluetooth.print(SetPoint); //print SetPoint to trace 2 
    bluetooth.print(".2*");
  }

int func_BTPIDValues(int P, float I, float D) //Prints PID variables to BlueTooth console  
  {  
    bluetooth.print("*A"); //select console 1
    bluetooth.print("P="); //print P to console
    bluetooth.println(P);

    bluetooth.print("I="); //print I to console 1
    bluetooth.println(I);   

    bluetooth.print("D="); //print D to console 1
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
