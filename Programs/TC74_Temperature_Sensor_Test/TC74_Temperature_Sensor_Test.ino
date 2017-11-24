//TC74_Test.ino
/*
********************************************************************************************************************************

Easy Build LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson - 22/11/2017

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.

This program reads the temperature from a TC74 sensor.

********************************************************************************************************************************
*/



#include <Wire.h>
#include <TC74_I2C.h>                          //https://github.com/Mario-H/TC74_I2C/blob/master/LICENSE.md

#include "HAB2_Board_Definitions.h"
//#include "Locator2_Board_Definitions.h"


TC74_I2C TC74(0x4c);  			                   //init with TC74 address

int temperature = 0;

#define TC74powersave 0                       //set to 1 for TC74 powersave mode



void loop() {

  if (TC74powersave)
  {
    TC74.NoPowersave();                        //turn off powesave mode
    Serial.println("TC74 powersave off");
  }

  temperature = TC74.ReadTemp();

  Serial.print ("Temperature = ");
  if (temperature != 128)
  {
    Serial.print(temperature);
    Serial.println("C");
  }
  else
  { 
    Serial.println(" TC74 Error");               //value of 128 degrees is an error
  }

  if (TC74powersave) 
  {
    TC74.Powersave();                           //put TC74 in low current mode, approx 5uA
    Serial.println("TC74 powersave on");
  }

  delay(3000);
}



void setup() {
  Serial.begin(38400);
  Serial.println("Starting TC74 Temperature Sensor Test");
  Serial.println();
  pinMode(GPSPOWER, OUTPUT);                    //in case power switching components for GPS breakout are fitted
  digitalWrite(GPSPOWER, LOW);

  TC74.Init();
}
