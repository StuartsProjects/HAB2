#define programname "HAB2_UBLOX_EchoGPS_Serial"
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
  
To Do:
  
*******************************************************************************************************************************
*/

/*
*******************************************************************************************************************************
  The purpose of this program is to check that a UBLOX Serial GPS is working. Characters are read from the GPS
  and sent to the Serial monitor at 115200 baud.
  The program can is configured to send the commands for a UBLOX GPS that should configure it out of GLONASS mode.
*******************************************************************************************************************************
*/


#include <Arduino.h>
#include <avr/pgmspace.h>

//#include "HAB2_Board_Definitions.h"         //select board type here  
//#include "PIHTracker3_Board_Definitions.h"         //select board type here
#include "Locator2_Board_Definitions.h"         //select board type here

#include "Program_Definitions.h"            //definitions for programs

const unsigned long GPS_WaitAck_mS = 2000;  //number of mS to wait for an ACK response from GPS
const byte GPS_attempts = 3;                //number of times the sending of GPS config will be attempted.
const byte GPS_Reply_Size = 12;             //size of GPS reply buffer
const unsigned int GPSBaud = 9600;          //baud rate of GPS
boolean GPS_Config_Error;  

#include <NeoSWSerial.h>                    //https://github.com/SlashDevin/NeoSWSerial  
NeoSWSerial GPSserial(GPSRX, GPSTX);        //this library is more relaible at GPS init than software serial

#define GPS_ALLOW_GPGSV                     //we want to see the GPGSV messages 
#include "UBLOX_SerialGPS.h"


void loop()                    
{
  while (GPSserial.available() > 0)
  Serial.write(GPSserial.read());
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

  pinMode(LED1, OUTPUT);                     //for Watchdog pulse input
  led_Flash(2, 500);

  pinMode(GPSPOWER, OUTPUT);                 //in case power switching components are fitted
  GPS_On(DoGPSPowerSwitch);                 //this will power the GPSon
  GPS_Setup();

  if (!GPS_CheckNavigation())                //Check that UBLOX GPS is in Navigation model 6
  {
    Serial.println();
    Serial.println(F("Warning GPS Error !"));
    Serial.println();
    led_Flash(100, 25);
  }
    
}


