This folder contains the current version of the LoRaTracker HAB2 programs.

The programs requre the installation of the current LoRaTracker Library files see here;

https://github.com/LoRaTracker/LoRaTracker-Library

The programs in here are;

LoRaTracker_HAB2_xxxxxx - The HAB tracker transmitter, supports two way LoRa control and tracking

FSKRTTY_HAB2_FSKRTTYONLY_xxxxxx - A FSK RTTY only version of the HAB tracker

HAB2_UBLOX_EchoGPS_I2C - A test program for the UBLOX GPS in I2C mode, checks that Navigation model 6 can be set

HAB2_UBLOX_EchoGPS_Serial - A test program for the UBLOX GPS in serial mode, checks that Navigation model 6 can be set

I2C_Scanner - a basic I2C scanner, reports any I2C devices found

Various third party Libraries may need to be installed, if not already present in your installation;

Flash-5 Library - https://github.com/mikalhart/Flash/releases
LowPower - https://github.com/rocketscream/Low-Power
TinyGPS++ - http://arduiniana.org/libraries/tinygpsplus/
NeoSWSerial  - https://github.com/SlashDevin/NeoSWSerial  

There is an issue with the Flash-5 library, changes to the Arduino IDE since the Library was published in 2014
cause a compile error which is reported as ‘prog_char’ does not name a type. The solution to this is to edit the 
Flash.h file in the Flash-5 Library. Add the following lines in Flash.h, just after the #include <avr/pgmspace.h>
i.e. around line 30

#if ARDUINO >= 150
typedef char prog_char __attribute__((__progmem__));
#endif

If you have any problems with the programs do check that you are using the latest revision of the programs, the date
is included as part of the program name.  