#define programname "HAB2_UBLOX_EchoGPS_I2C"
#define programversion "V1.0"
#define dateproduced "011017"
#define aurthorname "Stuart Robinson"


/*
*******************************************************************************************************************************
  Easy Build LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 1/10/17

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
  of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  This program reads the GPS via I2C. Stores the config settings for UBLOX GPS in Flash to save on RAM memory.

  To Do:

*******************************************************************************************************************************
*/

/*
*******************************************************************************************************************************
  The purpose of this program is to check that a UBLOX I2C GPS is working. Characters are read from the GPS
  and sent to the Serial monitor at 115200 baud.
  The program can is configured to send the commands for a UBLOX GPS that should configure it out of GLONASS mode.
*******************************************************************************************************************************
*/


#include <Arduino.h>
#include <avr/pgmspace.h>

#include "HAB2_Board_Definitions.h"       //select board type here  
//#include "Locator2_Board_Definitions.h"     //select board type here

#include "Program_Definitions.h"            //definitions for programs

const unsigned long GPS_WaitAck_mS = 2000;  //number of mS to wait for an ACK response from GPS
const byte GPS_attempts = 3;                //number of times the sending of GPS config will be attempted.
const byte GPS_Reply_Size = 12;             //size of GPS reply buffer
boolean GPS_Config_Error;

#define GPS_ALLOW_GPGSV                     //we want to see the GPGSV messages
#include "UBLOX_I2CGPS.h"

void loop()
{
  byte i;

  Wire.beginTransmission(GPSI2CAddress);
  Wire.write(0xFF);

  while (1)
  {
    Wire.requestFrom(GPSI2CAddress, 1);
    i = Wire.read();
    if (i != 0xFF)
    {
      Serial.write(i);
    }
  }
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  unsigned int index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void setup()
{
  pinMode(GPSPOWER, OUTPUT);	               //setup pin for GPS Power Control
  digitalWrite(GPSPOWER, LOW);

  Serial.begin(115200);                      //connect at 115200 so we can read the GPS fast enough and also spit it out
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));

  pinMode(LED1, OUTPUT);
  led_Flash(2, 500);

  Wire.begin();

  GPS_On(DoGPSPowerSwitch);                //this will power the GPSon
  GPS_Setup();

  if (!GPS_CheckNavigation())                //Check that UBLOX GPS is in Navigation model 6
  {
    Serial.println();
    Serial.println(F("Warning GPS Error !"));
    Serial.println();
    led_Flash(100, 25);
  }

}


