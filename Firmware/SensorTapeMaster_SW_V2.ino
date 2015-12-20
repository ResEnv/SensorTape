//this code is for the  SensorTapeMaster_v1 with build in USB transciever (ATmega32u4)
//Select the Arduino Leonardo board. (16MHz, 5V)
//Author: Artem Dementyev
//Date modified: September 13, 2015
//TODO: The neopixel light doesn't work on that board. Must be something with the clock being 16 Mhz
//uUisng arduino 1.0.5

//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

//Global definitions
#define BT_ENABLE      6
#define TAPE_POWER     5
#define LED_BLUE       8 
#define LED_RED        9
#define PIN_NEOPIXEL   13
#define P2P_SERIAL_RX  11
#define P2P_SERIAL_TX  10
#define NUMPIXELS      1 

//Global variables
boolean lookUpPresence[128];
int lookUpNumberOfSensors[128]; 
unsigned int deviceID = 200;
int x = 0;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
SoftwareSerial p2pSerial(P2P_SERIAL_RX,P2P_SERIAL_TX); //RX, TX

void setup() {                
  // initialize the digital pin as an output.
  pinMode(LED_BLUE, OUTPUT);   
  pinMode(BT_ENABLE, OUTPUT);
  pinMode(TAPE_POWER,OUTPUT);
  digitalWrite(BT_ENABLE,HIGH); //LOW - turn off
  digitalWrite(TAPE_POWER, HIGH);// HIGH - tape on 
  Serial.begin(115200);
  Serial.println("Connected");
  Wire.begin();  // Start I2C Bus as Master
  
  //This changes the I2C global bus speed to 400 kHz (max) 
  //#define TWI_FREQ_NUNCHUCK 400000L
  //#define CPU_FREQ 16000000L
  //TWBR = ((CPU_FREQ / TWI_FREQ_NUNCHUCK) - 16) / 2;
  
  delay(10);
  p2pSerial.begin(9600);
  
  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setPixelColor(0, pixels.Color(100,15,15)); // Moderately bright green color.
  pixels.show(); // This sends the updated pixel color to the hardware.

  //Start the peer to peer transmission. 
  delay(90); 
  int foo = 200;
  p2pSerial.write(foo);
  foo=1; 
  p2pSerial.write(foo); 
  delay(4000);//master should transmit only after slaves are ready

  //see how many slaves are present
  findWhoIsAround();
  
   pixels.begin(); // This initializes the NeoPixel library.
   pixels.setPixelColor(0, pixels.Color(10,30,10)); // Moderately bright green color.
   pixels.show(); // This sends the updated pixel color to the hardware.
   
}

void loop() {
  //Blink the LED
  digitalWrite(LED_BLUE, !digitalRead(LED_BLUE)); 

  //Poll slaves in order
  for(int i=0; i<128; i++) { 
    if(lookUpPresence[i]) { 
      if (i == 8 ) {
      requestFromDevice(i,18); //this is for the motion sensor 
    }
    else requestFromDevice(i,18);
    }
  }//end for

  //If the master wants to send something to a slave
  if (Serial.available() >= 3) { 
    delay(5);  //The communications sometimes crashes without this delay. 
    int whereToSend = Serial.read() *256; 
    whereToSend = whereToSend + Serial.read(); 
    byte command1 = Serial.read();
    byte command2 = Serial.read(); 
    byte command3 = Serial.read(); 
    delay(15); //The communications might crash without this delay 
    //Check just in case if the commands are below 128. This is the maximum that can be sent by java Char (-127 to 127) 
    if (command1 <128 && command2<128 & command3<128) { 
    sendByteToSlave(whereToSend, command1,command2,command3);
    }
    Serial.flush();
  }
}//end loop


void setToZeros() { 
  unsigned int deviceID = 200 ;
}//end setToZeros


void findWhoIsAround() { 
    for (int i=0; i<128; i++) { 
      requestFromDeviceDuringStart(i,14); 
      if(deviceID==i){ 
        Serial.print(i);
        Serial.println(" is present");
        lookUpPresence[i] = true; 
      } 
      else {
        lookUpPresence[i] = false;
      } 
      //delay(1000);
      digitalWrite(LED_RED, !digitalRead(LED_RED)); 
  }//end for
}//end findwhoIsAround



void requestFromDevice(int device, int bytes) { 
    unsigned char inputArray[bytes];
    
    unsigned char dataArray[bytes-2];
    
    setToZeros(); 
   if (Wire.requestFrom(device, bytes)>0) {  
      while(Wire.available())    
      { 
        unsigned char  c= Wire.read(); 
        inputArray[x] = c; 
        x++;   
      }//end while 
      
      x = 0; 
      
      for (int i=0; i<bytes-2; i++) { 
        dataArray[i] = inputArray[i];
      }
      
      byte *mypointer; 
      mypointer = dataArray;
      byte crc8 = CRC8(mypointer, 16);
      byte crc8in = inputArray[bytes-2];
      
      if(crc8 == crc8in) { 
        inputArray[bytes-2]= 0x02;
        inputArray[bytes-1]= 0x00;
      }
      else{ 
        inputArray[bytes-2]= 0x01;
        inputArray[bytes-1]= 0x00;
      }
        
      
     
      //Send data to the PC immidiately. 
      Serial.print("S");
      Serial.print(",");
      for(int i=0; i<bytes; i = i+2) { 
        Serial.print((inputArray[i+1]<<8) + inputArray[i]);
        if (i==bytes-2) { 
        //DO nothing
          }
        else
          Serial.print(",");
      }
      Serial.println(" ");
      deviceID = (inputArray[1]<<8) + inputArray[0];   
      
      
      /*
     for(int i=0; i<bytes-2; i++) { 
       Serial.print(dataArray[i],HEX); 
       Serial.print(","); 
     }
     Serial.print(",");
      Serial.print(crc8);
      Serial.print(","); 
      Serial.print(crc8in);
      Serial.println(" ");
      */
      
      
   }//end if 
 
 
     //Didnt find the device.
     else {
      Serial.print("S");
      Serial.print(",");
      Serial.print(device);
      Serial.print(",");
      for(int i=2; i<bytes; i = i+2) { 
        Serial.print(100);
        if (i==bytes-2) { 
        //DO nothing
          }
        else
          Serial.print(",");
      }
      Serial.println(" ");
      deviceID = device;  
     
   }
   
    
    
    
}

//This function is used to get information about the types of sensors. 
void requestFromDeviceDuringStart(int device, int bytes) { 
    unsigned char inputArray[bytes] ;
    setToZeros(); 
    if (Wire.requestFrom(device, bytes)>0) {   
      while(Wire.available())    // slave may send less than requested
      { 
        unsigned char  c= Wire.read(); 
        inputArray[x] = c; 
        x++;   
      }
      x = 0; 

      //Send to the PC immidiately 
      Serial.print("A");
      Serial.print(",");
      for(int i=0; i<bytes; i = i+2) { 
        Serial.print((inputArray[i+1]<<8) + inputArray[i]);
        if (i==bytes-2) { 
        //DO nothing
          }
        else
          Serial.print(",");
      }
      Serial.println(" ");
      deviceID = (inputArray[1]<<8) + inputArray[0];
      delay(5); //was 5 before
    }
} //end requestFromDeviceDuringStart

void sendByteToSlave(int address, uint8_t command1, uint8_t command2, uint8_t command3) { 
  Wire.beginTransmission(address); 
  Wire.write(command1); 
  Wire.write(command2); 
  Wire.write(command3); 
  Wire.endTransmission(); 
}  //end sendByteToSlave


//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//From here: 
///http://www.leonardomiliani.com/en/2013/un-semplice-crc8-per-arduino/
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}
