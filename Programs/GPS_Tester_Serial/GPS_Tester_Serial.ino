#define programname "GPS_Tester_Serial"
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
The purpose of this program is to check that a Serial GPS is working. Characters are read from 
the GPS and sent to the Serial monitor at 115200 baud.
 
A 9600 baud GPS is assumed. 

******************************************************************************************************
*/

#include <Arduino.h>
#include <avr/pgmspace.h>

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


void setup()
{
  Serial.begin(115200);                        //connect at 115200 so we can read the GPS fast enough and also spit it out
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  GPSserial.begin(GPSBaud);                    //Startup soft serial

}


