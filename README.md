#SensorTape Project 


##Introduction 
SensorTape is a sensor network in a form factor of a tape. It allows users to intuitively create large sensor networks. See mode details in the paper from [UIST](http://artem.dementyev.us/wp-content/uploads/2013/10/Sensor_tape_UIST_2015_rev9_FINAL.pdf) conference and a video 
This repository contains all the hardware and software design files to make SensorTape. 

![MediaLab Logo](/images/tape.jpg)

##Content
Master_hw/ - Hardware files for the master board in Altium format. Can be used to make the board

Slave_hw/ - Hardware files for the tape itself, refered here as the slave. This is the flexible circuit version of the tape 

Prog_adapter_hw/ -  Hardware files for the programming board, use it to program both the slave and the master. 

libraries/ - Collection of the Arduino libraries for the master and the slave. Those libraries are required 

Firmware/ - Firmware for both the master and the slave

SensorTape_Demo_V1/ - Processing sketch for a simple visualization demo. Demonstrates the basics of interfacing with a PC. 


##Firmware 
Add the arduino libraries from the libraries/ folder here to the worspace libraries folder. 
You will need the following libraries Adafruit_NeoPixel, I2C_dev, MPU6050, and VL6180X_I2C_Soft

you will need a programmer and an adapter board (Prog_adapter_hw/) to load the software. I used AVRISP mKII.
you will need to load the bootloader first into the boards. This will set the fuses and the clock frequency. Use the Arduino Micro (3.3V, 8 MHz) for the slaves and Arduino leonardo for the master. Set those before loading the bootloader

##PC software
I used Processing 3. You will need one external library (toxiclibs-complete-0020), it is in the libraries folder. 

##Slave
The slaves are basically the tape. The tape is made of strips with 6 slave nodes. You will need a flexible PCB factory to manufacture. The factories in china are innexpensive and have good quality. They can be found on alibaba.com. Also it is better to get it assembled profesionally, since hand assembly is difficult. I had a lot of problems with the packaging of proximity sensor melting in my reflow owen. 

The programming connector can be hard to attach on the first try, since it is just pressfitted. 

When starting the tape wait about a minute for the IMUs to calibrate, otherwise the orientaion data will drift

##Master
Master moves the data from the slaves to the PC. I am using an ATMega32u4 microcontroller, which works as a USB modem. 

##Known Issues
1. Proximity sometimes dont work if tape is restarted. I suspect that they don't have time to fully discharge. When turning the tape on and off or restarting, wait some time to turn it back on. 
2. The number of nodes is set pn the PC side in the current version. So the nodes dont need to be reset each time PC software is restarted. 


##Aknowledgements
MIT Media Lab, Nan-Wei Gong, Cindy Kao, Nan Zhao, Felix Heibeck, National Sceince Foundation. 
![MediaLab Logo](/images/logo.png)