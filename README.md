# CC11XX Library that supports full SPI

## Description:
This is a library that I'm using to get Full SPI working with Transactions. It currently works with a Teensy 3.1/3.2. I'm going to also test this on other platforms like Cortex M0's, Atmel SAMDxx, Espressif ESP8266, Espressif ESP32, etc. I'm up to any recommendations on other platforms to make it compatible with. 

I don't take full credit for this project as I have just taken other peoples work, and started experimenting to make it work correctly. I did find other SPI libraries for the CC11xx but none seem to work, unstable, or outdated. My goal is to get this working fully. I have it working today on a Teensy 3.1/3.2 and other platforms listed below, but that is all I have tested it on. 

I have added a Receive Example, it's set for 315MHz right now. It can be changed easily to any of the CC11xx supported frequencies. I'm using some cheap modules purchased off eBay. They were originally designed to work at 433MHz, but I am able to send and receive in the 315MHz range. I'm sure it's not 100% ideal, but does work. I have not tested higher ranges, but will assume it works, but will try later.

The receive example is mainly for devices that don't have Multi-Core. I will build a seperate one for the Espressif ESP32 in the near future. It won't take much to make it work. Eventually I will consolidate all examples down to a single RX example, and a single TX Example.

Most of my random isssues I was experiencing have all been cleaned up, and no longer happen.

I've add what I call cc11xxOptions, which gives you the ability to assign the Chip Select, GDO0, and GDO2 pins right from the sketch. This is the step forward (I think) to being able to run multiple radios. 

Example:
  ```bash
  #include <SPI.h>
  #include <cc1101.h>
  #include <ccpacket.h>

  CC1101 cc1101;
  
  const int dataReadyPin = 2;
  const int chipSelectPin = 10;
  ...
  SPI.begin();
  pinMode(dataReadyPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);
  // Call your config options before you initiate your radio
  cc1101.cc11xx_cfg.CS_pin = chipSelectPin;
  cc1101.cc11xx_cfg.GDO0_pin = dataReadyPin;
  
  cc1101.init();
  ...
  ```


## Features:
- Arduino code to get everything running
- Full SPI Support
- Ability to define Chip Select, GDO0, and GDO2 pins in the sketch.

## Known Working MCU
- Teensy 3.1/3.2
- Teensy 3.5/3.6
- Teensy LC
- Espressif ESP32


## Future:
- Compatibility
- Multiple CC11xx modules (Maybe)

  