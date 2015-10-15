
//This is for tape flexible pcb version of the tape (SensorTapeFlex_V1)
//Use Arduino Micro (8MHz, 3.3V)
//Author: Artem Dementyev
//Modified on Septeber 9,2015

//I2C pins are switched from the Rigid PCB Version: 
//CHANGE the I2Cdev.h  SDA_PIN and SCL_PIN 

 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "VL6180X_I2C_Soft.h"

MPU6050 mpu;
SoftwareSerial p2pSerial(9,10); //RX, TX

#define PIN_RGB_LED    7
#define LED_BLUE       6
#define LED_RED        5
#define ADC_LIGHT_PIN  A2
//#define CUT_1     A1      
//#define CUT_2     8
#define NUMPIXELS      1  
#define VL6180X_ADDRESS 0x29


bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
//global variables
int numberOfSensors = 3;
int deviceType = 1;
int deviceID = 0; 
int cut1pin = A1; 
int cut2pin = 8; 
boolean startUp = true; 
int incomingByte = 0;
byte val1; 
byte val2;  
int lightAnalogRead = 2000; 
int thermistorAnalogRead = 3000; 
int positionAnalogRead = 4000;
String valueString = "a"; 
byte valAll[1]; 
uint8_t distanceReading =0x10; 
uint8_t masterReceive[3] = {0x00,0x00,0x00} ;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);
VL6180x distanceSensor(VL6180X_ADDRESS);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE FOR IMU        ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    //set pins  
    pinMode(LED_BLUE, OUTPUT);   
    pinMode(LED_RED, OUTPUT); 
    digitalWrite(LED_BLUE, HIGH); 
  
    SPI.begin();
    p2pSerial.begin(9600);
    delay(50); 
  
    //Wait until data from peer to peer network
    while(true){ 
      if(p2pSerial.available()>1) { 
        if(p2pSerial.read()==200) { 
          incomingByte = p2pSerial.read();
          //assign address from incoming data  
          deviceID=incomingByte; 
  
          //send data to the next device
          p2pSerial.write(200);
          p2pSerial.write(incomingByte+1);
          digitalWrite(LED_BLUE, LOW);  
          break; 
        }
      }
    }//end while
    p2pSerial.end(); 
  
    I2Cdev::i2c_init();

    //Start imu and proximity initialization
    mpu.initialize();
    distanceSensor.VL6180xInit();
  
    devStatus = mpu.dmpInitialize();
    distanceSensor.VL6180xDefautSettings();
    distanceSensor.VL6180x_setRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET  , 0x00); 
    distanceSensor.VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x01); //Start the 100ms continous operation.
   
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
    
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); 
    
    Wire.begin(deviceID); // Start I2C Bus as a Slave (Device Number 2)
    Wire.onRequest(requestEvent); // register event to send data to master (respond to requests)
    Wire.onReceive(receiveEvent); //register even to receice data from master
    attachInterrupt(0, dmpDataReady, RISING);
     
    pixels.begin(); // This initializes the NeoPixel library.
    pixels.setPixelColor(0, pixels.Color(0,10,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.    
}//end setup

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
      //This is the MAIN LOOP
     while (!mpuInterrupt && fifoCount < packetSize) {
            //DO nothing, wait for interrupts 
      }//end while
    
    //STEPPING OUT OF THE LOOP FOR INTERRUPT  
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    //    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

       
        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose

       //get data from priximity sensor 
       if(!startUp) {  
         noInterrupts();
         char range_status = distanceSensor.VL6180x_getRegister(VL6180X_RESULT_INTERRUPT_STATUS_GPIO) & 0x07;
         // wait for new measurement ready status
         if (range_status == 0x04) {
             distanceReading = distanceSensor.VL6180x_getRegister(VL6180X_RESULT_RANGE_VAL); 
             distanceSensor.VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
             distanceSensor.VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode
             digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
         }//end if
         interrupts();
       }
    }//end else if 
}//end loop

//This function assembles a reply to a master's request
void requestEvent()
{
    //The communication sends the number of sensors and type of device. 
    if (startUp) { 
        byte a = deviceID & 0xFF; //Use this format to send 16bit integer to java. 
        byte b = (deviceID >>8 ) & 0xFF;
        
        byte c = numberOfSensors & 0xFF; ; 
        byte d = (numberOfSensors >>8 ) & 0xFF;
        
        byte e = deviceType & 0xFF; ; 
        byte f = (deviceType >>8 ) & 0xFF;
        
        int tempread = digitalRead(cut1pin); 
        byte g = tempread & 0xFF; ; 
        byte h = (tempread >>8 ) & 0xFF;     
        
        tempread = digitalRead(cut2pin); 
        byte i = tempread & 0xFF; ; 
        byte j = (tempread >>8 ) & 0xFF;   
        
        tempread = digitalRead(cut2pin); 
        byte k = tempread & 0xFF; ; 
        byte l = (tempread >>8 ) & 0xFF; 
        
        tempread = digitalRead(cut2pin); 
        byte m = tempread & 0xFF; ; 
        byte n = (tempread >>8 ) & 0xFF; 
        
        byte All [] = {a,b,c,d,e,f,g,h,i,j,k,l,m,n};  
           
        Wire.write(All, 14); // respond with message 
        startUp = false;
    } 

    //Otterwise, send data from the on-board sensors. 
    else { 
        digitalWrite(LED_RED, HIGH);
        lightAnalogRead = analogRead(ADC_LIGHT_PIN); // Read light value 
        thermistorAnalogRead = readInternalTemperature(); //Read thermistor valie
        positionAnalogRead = analogRead(A1); //Read position
        
        byte a = deviceID & 0xFF;
        byte b = (deviceID >>8 ) & 0xFF;
        
        byte c = lightAnalogRead & 0xFF; ; 
        byte d = (lightAnalogRead >>8 ) & 0xFF;
     
        byte e = thermistorAnalogRead & 0xFF; ; 
        byte f = (thermistorAnalogRead >>8 ) & 0xFF;
        
        byte g = distanceReading & 0xFF;
        byte h = (0x00 >>8 ) & 0xFF;
       
        byte All [] = {a,b,c,d,e,f,g,h, 
                      fifoBuffer[1], fifoBuffer[0], 
                      fifoBuffer[5], fifoBuffer[4], 
                      fifoBuffer[9], fifoBuffer[8],
                      fifoBuffer[13], fifoBuffer[12]};              
        Wire.write(All, 16); 
        digitalWrite(LED_RED, LOW);    
    }//end else        
}//end requestEvent

//This function sets led color based on master command
void receiveEvent(int howMany)
{
  if (Wire.available()) {    
      masterReceive[0] = Wire.read(); 
      masterReceive[1] = Wire.read(); 
      masterReceive[2] = Wire.read(); 
  }
     pixels.setPixelColor(0, pixels.Color(masterReceive[0],masterReceive[1],masterReceive[2])); 
     pixels.show();
}

int readInternalTemperature() {
    int result;
    // Read temperature sensor against 1.1V reference
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA,ADSC));
    result = ADCL;
    result |= ADCH<<8;
    //result = (result - 125) * 1075;
    return result;
}

