#define programname "I2C_Scanner"
#define dateproduced "7/8/2017"
#define aurthorname "Stuart Robinson"

/*
********************************************************************************************************************************

Easy Build LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson - 7/8/2017

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.

This program scans each address on the I2C bus and reports if a device was found.

BME280 0x76
UBLOX GPS 0x42


********************************************************************************************************************************
*/


#include <Arduino.h>
#include <Wire.h>
#include "I2C_Scanner.h"
#include "HAB2_Board_Definitions.h"

unsigned int scancount;

void loop()
{
  scancount++;
  Serial.println();
  Serial.print(scancount);
  Serial.print(" ");
  run_I2CScan();
  delay(1000);
}



void setup()
{
  Serial.begin(38400);                                   //setup Serial console ouput
  Serial.println(F(programname));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();
 }


