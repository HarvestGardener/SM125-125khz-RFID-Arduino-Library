//Example code demonstrating the use of the SM125 library
//by HarvestGardener; free to use and distribute

#include <NewSoftSerial.h>
#include <SM125.h>

//define the pins the SM125 is connected to
#define rxPin 2
#define txPin 3

//Integer to hold commands received from serial monitor
int received = 0;

// initialize the library with the numbers of the interface pins
SM125 myRFID =  SM125(rxPin, txPin);

void setup(){
  //Start serial
  Serial.begin(9600);
  Serial.println("Sample code to demonstrate use of SM125 125khz RFID Library");
  
  Serial.println("RFID Start");
  
  
}

void loop(){
  //code to take in serial commands and pass them to the RFID IC
  if(Serial.available() == 1)
  {
    received = Serial.read();
    switch (received)
    {
      //Read Firmware version from RFID IC, command will print it to the serial port
      case '1': 
      {
	 Serial.println("Read Firmware Version");
         myRFID.cmdReadFirmwareVersion();
         break;
      }
      
      //Read an EM4102 Card then stop
      case '2': 
      {
        Serial.println("Read an EM4102 Card");
        
        //0x03 = EM4102 Mode – Parity decoded – Manchester RF/64
        //0x02 = EM4102 Default
        myRFID.cmdRead(0x03,0x02);
        
        //Print the card data to serial
        myRFID.printCmdResponse();
        
        //stop reading
        myRFID.cmdStopRead();
	break;
      }
      
      //Read the full contents of a T55x7 Card
      case '3':
      {
        Serial.println("Read T5557 - All 7 Blocks");
        
        //0x01 = Byte Track Mode – Manchester RF/64
        //0x07 = Read Blocks 1-7
        myRFID.cmdRead(0x01, 0x07);
        
        //read the card data, returned as a byte array
        byte* T55x7Response = myRFID.getSM125Response();
        
        //Print response to serial
        int len = (int)T55x7Response[0];
        for(int i=1; i<len; i++) {
          Serial.print(T55x7Response[i], HEX);
        }
        Serial.print('\n');

        //stop reading
        myRFID.cmdStopRead();
        break;
      }
      
      case '4':
      {
        Serial.println("Write T5557 block 2");
        
        //Set the parameters for the T5557 card
        myRFID.cmdSetT5557ProgParam();
        
        //0x02 = Block 2
        //0xAA = Data to be written, bytes 1-4
        myRFID.cmdWrite(0x02, 0xAA, 0xAA, 0xAA, 0xAA);
        
        //wait 500ms to finish write operations
        delay(500);
        
        Serial.println("Write complete");
        break;
      }
      
      case '5': //This command takes a while
      {
        Serial.println("T5557 reset");
        
        //send reset command
        myRFID.cmdResetT5557Tag();
        
        Serial.println("Reset complete");
        break;
      }
    }
 }
}
