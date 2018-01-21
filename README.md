# CC11XX Library that supports full SPI

## Description:
This is a library that I'm using to get Full SPI working with transactions. It currently works with a Teensy 3.1/3.2. This will be tested on other platforms like Cortex M0's, Atmel SAMDxx, Espressif ESP8266, Espressif ESP32, etc. I'm open to requests to create compatibility with other platforms. 

I cannot take full credit for this project, as I have leveraged other peoples' work-in-progress and started experimenting to make it work correctly. I did find other SPI libraries for the CC11xx but either they were unstable, outdated, or just did not work properly. My goal is to get this working fully. I have it working today on a Teensy 3.1/3.2 and have tested on the platforms listed below. 

I have added a Receive Example, it's set for 315MHz right now. It can be easily changed to any of the CC11xx supported frequencies. I'm using some cheap modules purchased off eBay. They were originally designed to work at 433MHz, but I am able to send and receive in the 315MHz range. I'm sure it's not 100% ideal; however it does work. I have not tested higher ranges yet.

The Receive Example is mainly for devices that do not have Multi-Core. I will build a separate example for the Espressif ESP32 in the near future. Eventually, I will consolidate all examples down to a single RX example, and a single TX Example.

Most random issues experienced (e.g. not always sending data) have been cleaned up and no longer happen.

I've add what I call cc11xxOptions, which gives you the ability to assign the Chip Select, GDO0, and GDO2 pins right from the sketch. This is the step forward (I think) to be able to run multiple radios. 

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
- Adafruit Feather M0
- Espressif ESP32


## Future:
- Compatibility
- Multiple CC11xx modules (Maybe)

  