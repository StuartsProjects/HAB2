#define programname "UBLOX_GPS_Tester_Serial"
#define programversion "V1.0"
#define dateproduced "110717"
#define aurthorname "Stuart Robinson"


/*
**************************************************************************************************

LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson - 11/07/2017

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.


To do:


Changes:


******************************************************************************************************
*/

/*
******************************************************************************************************
The purpose of this program is to check that a UBLOX Serial GPS is working. Characters are read from 
the GPS and sent to the Serial monitor at 115200 baud.
 
A 9600 baud GPS is assumed and the program is configured to send the commands for a UBLOX GPS that
should take it out of GLONASS mode.

******************************************************************************************************
*/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Flash.h>                           //the FLASH library is needed to store the UBLOX config commands in FLASH rther than RAM

#define GPSTX A2                             //pin number for TX output - RX into GPS
#define GPSRX A3                             //pin number for RX input - TX from GPS
#define GPSBaud 9600                         //GPS baud rate

#include <SoftwareSerial.h>
SoftwareSerial GPSserial(GPSRX, GPSTX);     //Create the serial connection to the GPS device


void loop()                    
{
  while (GPSserial.available() > 0)
  Serial.write(GPSserial.read());
}


void config_UBLOXGPS()
{
  //each config sentence starts in the array with the nuimber of bytes to send
  //if the number of bytes to send is 0, that is the end of messages to send 
  
  byte j;
  byte byteread;
  byte Messagesize;
  int ptr;
  
  Serial.println(F("Config Ublox GPS"));

  FLASH_ARRAY(byte, GPSFlash,  
              0x15, 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98, //clear current config

              0x14, 0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x8F, 0xB2,  //glonass off
              

              0x10, 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, //GPGLL off

              0x10, 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46, //GPGLL off

              0x10, 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31, //GPGLS off

              //0x10, 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38, //GPSGSV off

              0x2C, 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, //Flight mode on
              0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC,

              0x15, 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA9, //save current config
              

              0x00);  //messages end

  ptr = 0;

  do
  {
    Messagesize = GPSFlash[ptr];

    ptr++;

    for (j = 1; j <= Messagesize; j++)
    {
      byteread = GPSFlash[ptr];
      GPSserial.write(byteread);
      ptr++;
    }
    Messagesize = GPSFlash[ptr];
    delay(50);
  }
  while  (Messagesize != 0x00);  //0x00 message size terminates list
}




void setup()
{
  Serial.begin(115200);                        //connect at 115200 so we can read the GPS fast enough and also spit it out
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  GPSserial.begin(GPSBaud);                    //Startup soft serial
  
  config_UBLOXGPS();                           //configure GPS out of GLONASS mode.    
 
}


