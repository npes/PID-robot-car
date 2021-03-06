/*
  Example Bluetooth Serial Passthrough Sketch
 by: Jim Lindblom
 SparkFun Electronics
 https://learn.sparkfun.com/tutorials/using-the-bluesmirf
 date: February 26, 2013
 license: Public domain

 This example sketch converts an RN-42 bluetooth module to
 communicate at 9600 bps (from 115200), and passes any serial
 data between Serial Monitor and bluetooth module.

 BTA=0006667D9784 (OUR bluetooth address)
 Get commands: D and E
 Set commands:  SN,<name> (set bluetooth device name)
                SF,1 (reset to default)
                ST,0 (config timer off)
 Action commands:
                I,<value> (search for other Bluetooth modules in range. value=search time)

 */
#include <SoftwareSerial.h>  

int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3
int input = 5;

String colors[]={"R0G0B0","R0G150B0"}; //App Light colours for LOW and HIGH
String receive_chars="abcdefghijkl"; //so app knows which data goes where
int interval=100; //Gives the serial link and app a chance to process data
float voltage; //Assumes 0-1023 range over 5V

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void setup()
{
  //Serial.begin(9600);  // Begin the serial monitor at 9600bps

  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
  //Initiate Digital pins as Inputs
  for(int i=5;i<=13;i++) pinMode(i, INPUT); 
  

}

void loop()
{
      for (;;)
      {
      //Read Digital Pins and Send results over Bluetooth
        for(int i=2;i<=13;i++){
        Serial.print("*"+String(receive_chars.charAt(i-2))+colors[digitalRead(i)]+"*");
        }
      delay(100);
      }
  // and loop forever and ever!
}
