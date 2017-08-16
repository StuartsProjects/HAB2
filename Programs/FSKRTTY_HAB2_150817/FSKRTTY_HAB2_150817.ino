//**************************************************************************************************
// Note:
//
// Make changes to this Program file at your peril
//
// Configuration changes should be made in the HAB2_Settings file not here !
//
//**************************************************************************************************

#define programname "FSKRTTY_HAB2_150817"
#define aurthorname "Stuart Robinson"

#include <Arduino.h>
#include <avr/pgmspace.h>

#include "HAB2_Settings.h"
#include "Program_Definitions.h"

/*
**************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 15/08/17

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  Location payload is constructed thus;

  PayloadID,Sequence,Time,Lat,Lon,Alt,Sats,SupplyVolts,Temperature,Resets,Config0byte,StatusByte,hertzoffset,GPSFixTime,Checksum
  0           1      2   3   4   5    6      7            8         9        10          11         12          13        14

  To Do:
  GPSoff time may be incorrect at initial lock

  Changes:

**************************************************************************************************
*/

//char RemoteControlNode;
//char ThisNode;

//int CalibrationOffset;

//unsigned long TrackerMode_Frequency;           //frequencies, and other parameters, are copied from memory into RAM.
//byte TrackerMode_Power;

byte TRStatus;                                      //used to store current status flag bits
byte Current_TXconfig1;                        //sets the config of whats transmitted etc
byte Cmd_WaitSecs;
byte stripvalue;
byte sats;                                          //either sats in view from GPGSV or sats from NMEA fix sentence
int internal_temperature;


//unsigned int Sleepsecs;                        //seconds for sleep at end of TX routine
//unsigned int WaitGPSFixSeconds;                //in flight mode, default time to wait for a fix
//unsigned int FSKRTTYbaudDelay;                 //dealy used in FSKRTTY routine to give chosen baud rate

float Fence_Check_Lon;                              //used for fence check

float TRLat;                                        //tracker transmitter co-ordinates
float TRLon;
unsigned int TRAlt;
byte keypress;

#include Board_Definition                            //include previously defined board file
#include Memory_Library                              //include previously defined Memory Library

#include <SPI.h>
#include <LowPower.h>                                //https://github.com/rocketscream/Low-Power

#include <TinyGPS++.h>                               //http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                     //create the TinyGPS++ object
TinyGPSCustom GNGGAFIXQ(gps, "GNGGA", 5);            //custom sentences used to detect possible switch to GLONASS mode

#ifdef USE_SOFTSERIAL_GPS
#include <NeoSWSerial.h>                             //https://github.com/SlashDevin/NeoSWSerial  
NeoSWSerial GPSserial(GPSRX, GPSTX);                 //this library is more relaible at GPS init than software serial
#endif

#include GPS_Library                                 //include previously defined GPS Library 

#include "Voltage_Temperature.h"
#include "LoRa3.h"
#include "FSK_RTTY2.h"


void loop()
{
  Serial.println();
  Serial.println();
  GPSFixTime = 0;
  internal_temperature = (int) read_Temperature();    //read temp just after sleep, when CPU is closest to ambient

#ifndef DEBUGNoGPS
  gpsWaitFix(WaitGPSFixSeconds, SwitchOn, LeaveOn);
#ifdef USE_SERIALGPS
  GPSserial.end();                                   //dont want soft serial running for now, it interferes with RTTY timing
#endif
#endif

#ifdef DEBUGNoGPS
  Serial.println(F("No GPS Test Mode"));
  setStatusByte(NoGPSTestMode, 1);
  TRLat = TestLatitude;
  TRLon = TestLongitude;
  TRAlt = (unsigned int) TestAltitude;
#endif

  do_Transmissions();                                             //yes, so do transmissions

  digitalWrite(lora_NSS, HIGH);                                   //take NSS line high, makes sure LoRa device is off
  sleepSecs(Loop_Sleepsecs);                                      //this sleep is used to set overall transmission cycle time
}



void do_Transmissions()
{
  //this is where all the transmisions get sent
  byte index, Count;

  pulseWDI();

  incMemoryULong(addr_SequenceNum);                                  //increment sequence number
  Count = buildHABPacket(lora_TXBUFF);

  Serial.println(F("Send FSKRTTY"));

  lora_Setup();                                                 //resets then sets up LoRa device
  lora_DirectSetup();                                           //set for direct mode
  lora_SetFreq(TrackerMode_Frequency, CalibrationOffset);
  lora_TXONDirect(10);
  Start_FSKRTTY(FSKRTTYRegshift, FSKRTTYleadin, FSKRTTYpips);

  for (index = 1; index <= sync_chars; index++)
  {
    Send_FSKRTTY('$', FSKRTTYbaudDelay);
  }

  for (index = 0; index <= Count; index++)
  {
    Send_FSKRTTY(lora_TXBUFF[index], FSKRTTYbaudDelay);
  }
  Send_FSKRTTY(13, FSKRTTYbaudDelay);                           //finish RTTY with carriage return
  Send_FSKRTTY(10, FSKRTTYbaudDelay);                           //and line feed
  digitalWrite(LED1, LOW);                                      //make sure LED off
}


byte buildHABPacket(byte *lora_TXBUFF)                           //note that expects a char buffer, so this routine might not work without the -
{
  //build the long tracker payload
  unsigned int index, j, CRC, resets, runmAhr;
  int volts;
  byte Count;
  char LatArray[12], LonArray[12];

  unsigned long sequence;

  sequence = Memory_ReadULong(addr_SequenceNum);                 //sequence number is kept in non-volatile memory so it survives resets
  resets =  Memory_ReadUInt(addr_ResetCount);                    //reset count is kept in non-volatile memory so it survives resets

  Serial.print("Resets ");
  Serial.println(resets);
  Serial.print("Internal Temperature ");
  Serial.println(internal_temperature);

  volts = read_SupplyVoltage();

  sats = gps.satellites.value();
  dtostrf(TRLat, 7, 5, LatArray);                                //format is dtostrf(FLOAT,WIDTH,PRECISION,BUFFER);
  dtostrf(TRLon, 7, 5, LonArray);                                //converts float to character array
  memset(lora_TXBUFF, 0, sizeof(lora_TXBUFF));                   //clear array

  Count = snprintf(lora_TXBUFF,
                   Output_len_max,
                   "$$$$%s,%lu,%02d:%02d:%02d,%s,%s,%d,%d,%d,%d,%d,%d,%d",
                   Flight_ID,
                   sequence,
                   gps.time.hour(),
                   gps.time.minute(),
                   gps.time.second(),
                   LatArray,
                   LonArray,
                   TRAlt,
                   sats,
                   volts,
                   internal_temperature,
                   resets,
                   Current_TXconfig1,
                   TRStatus
                  );

  Count = strlen(lora_TXBUFF);                    //how long is the array ?

  CRC = 0xffff;                                   //start value for CRC16

  for (index = 4; index < Count; index++)         //element 4 is first character after $$ at start
  {
    CRC ^= (((unsigned int)lora_TXBUFF[index]) << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }

  lora_TXBUFF[Count++] = '*';
  lora_TXBUFF[Count++] = Hex((CRC >> 12) & 15);      //add the checksum bytes to the end
  lora_TXBUFF[Count++] = Hex((CRC >> 8) & 15);
  lora_TXBUFF[Count++] = Hex((CRC >> 4) & 15);
  lora_TXBUFF[Count] = Hex(CRC & 15);
  return Count;
}


char Hex(char lchar)
{
  //used in CRC calculation in buildHABPacket
  char Table[] = "0123456789ABCDEF";
  return Table[lchar];
}


void printPayload(byte lCount)
{
  byte index;
  for (index = 0; index <= lCount; index++)
  {
    Serial.write(lora_TXBUFF[index]);
  }
}


void sleepSecs(unsigned int LNumberSleeps)
{
  unsigned int i;

  Serial.print(F("zz "));
  Serial.println(LNumberSleeps);
  Serial.flush();                                      //let print complete
  GPSserial.end();                                     //we dont want GPS interfering with sleep
  digitalWrite(lora_NSS, HIGH);                        //ensure LoRa Device is off

  for (i = 1; i <= LNumberSleeps; i++)
  {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);    //sleep 1 second
    pulseWDI();
  }

}


void incMemoryULong(unsigned int laddress)
{
  unsigned long val;
  val = Memory_ReadULong(laddress);
  val++;
  Memory_WriteULong(laddress, val);
}


byte readConfigByte(byte bitnum)
{
  return bitRead(Current_TXconfig1, bitnum);
}


void setConfigByte(byte bitnum, byte bitval)
{
  //program the config byte
  if (bitval == 0)
  {
    bitClear(Current_TXconfig1, bitnum);
  }
  else
  {
    bitSet(Current_TXconfig1, bitnum);
  }
  Memory_WriteByte(addr_Default_config1, Current_TXconfig1);

  if (bitval)
  {
    Serial.print(F("Set Config Bit "));
  }
  else
  {
    Serial.print(F("Clear Config Bit "));
  }
  Serial.println(bitnum);
}


void setStatusByte(byte bitnum, byte bitval)
{
  //program the status byte
  if (bitval == 0)
  {
    bitClear(TRStatus, bitnum);
  }
  else
  {
    bitSet(TRStatus, bitnum);
  }

  if (bitval)
  {
    Serial.print(F("Set Status Bit "));
  }
  else
  {
    Serial.print(F("Clear Status Bit "));
  }

  Serial.println(bitnum);
}


void pulseWDI()
{
  //if the watchdog is fitted it needs a regular pulse top prevent reset
  //togle the WDI pin twice
  digitalWrite(WDI, !digitalRead(WDI));
  delayMicroseconds(1);
  digitalWrite(WDI, !digitalRead(WDI));
}


boolean gpsWaitFix(unsigned long waitSecs, byte StartState, byte LeaveState)
{
  //waits a specified number of seconds for a fix, returns true for good fix
  //StartState when set to 1 will turn GPS on at routine start
  //LeaveState when set to 0 will turn GPS off at routine end, used perhaps when there is no fix

  unsigned long endwaitmS, millistowait, currentmillis;
  pulseWDI();
  byte GPSByte;

  if (StartState == 1)
  {
    GPS_On(UseGPSPowerControl);
  }
  else
  {
    GPS_On(DoNotUseGPSPowerControl);
  }

  Serial.print(F("Wait Fix "));
  Serial.print(waitSecs);
  Serial.println(F(" Secs"));

  currentmillis = millis();
  millistowait = waitSecs * 1000;
  endwaitmS = currentmillis + millistowait;

  while (millis() < endwaitmS)
  {

    do
    {
      GPSByte = GPS_GetByte();
      if (GPSByte != 0xFF)
      {
        gps.encode(GPSByte);
      }
    }
    while (GPSByte != 0xFF);

    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    {
      Serial.println(F("GPS Fix"));
      TRLat = gps.location.lat();
      TRLon = gps.location.lng();
      TRAlt = (unsigned int) gps.altitude.meters();

      if (readConfigByte(GPSPowerSave))
      {
        GPS_Off(UseGPSPowerControl);
      }
      else
      {
        GPS_Off(DoNotUseGPSPowerControl);
      }

      setStatusByte(GPSFix, 1);
      pulseWDI();
      return true;
    }
  }

  //if here then there has been no fix and a timeout
  setStatusByte(GPSFix, 0);                       //set status bit to flag no fix
  Serial.println(F("No Fix"));

#ifdef UBLOX
  if (GNGGAFIXQ.age() < 2000)                     //check to see if GLONASS has gone active
  {
    Serial.println(F("GLONASS !"));
    setStatusByte(GLONASSisoutput, 1);

    GPS_SetGPMode();
    sleepSecs(1);
    GPS_SetCyclicMode();
    sleepSecs(1);
  }
  else
  {
    setStatusByte(GLONASSisoutput, 0);            //GLONASS not detected
  }
#endif

  if (LeaveState == 0)
  {
    //no fix but gpsWaitFix called with gpspower to be turned off on exit
    GPS_Off(UseGPSPowerControl);
  }
  else
  {
    //no fix but gpsWaitFix called with gpspower to be left on at exit
    GPS_Off(DoNotUseGPSPowerControl);
  }

  pulseWDI();
  return false;
}



void display_current_frequency()
{
  float freq_temp;
  freq_temp = lora_GetFreq();
  Serial.print(F("Frequency "));
  Serial.print(freq_temp, 3);
  Serial.println(F("MHz"));
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  //flash LED to show tracker is alive
  unsigned int index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}

void Clear_All_Memory()
{
  //clears the whole of memory, normally 1kbyte
  Serial.print(F("Clear All Memory"));
  Memory_Set(addr_StartMemory, addr_EndMemory, 0);
}


void setup()
{
  unsigned long int i;
  unsigned int j;

  pinMode(LED1, OUTPUT);                  //for PCB LED
  pinMode(WDI, OUTPUT);                   //for Watchdog pulse input

  led_Flash(2, 500);

  Serial.begin(38400);                    //Setup Serial console ouput

  Serial.println(F(programname));
  Serial.println(F(aurthorname));

#ifdef ClearAllMemory
  Clear_All_Memory();
#endif

  pinMode(GPSPOWER, OUTPUT);              //in case power switching components are fitted
  GPS_On(UseGPSPowerControl);              //this will power the GPSon

#ifdef USE_SERIALGPS
  GPSserial.end();                        //but we dont want soft serial running for now, it interferes with the LoRa device
#endif

  pinMode(lora_NReset, OUTPUT);           //LoRa device reset line
  digitalWrite(lora_NReset, HIGH);

  pinMode (lora_NSS, OUTPUT);             //set the slave select pin as an output:
  digitalWrite(lora_NSS, HIGH);

  SPI.begin();                            //initialize SPI
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  Serial.print(F("Config1 "));
  Serial.println(Default_config1, BIN);

  j = Memory_ReadUInt(addr_ResetCount);
  j++;
  Memory_WriteUInt(addr_ResetCount, j);
  Serial.print(F("Resets "));
  Serial.println(j);
  Serial.print(F("Sequence "));
  i = Memory_ReadULong(addr_SequenceNum);
  Serial.println(i);

  lora_Setup();

  if (!lora_CheckDevice())
  {
    led_Flash(40, 50);                                                //long medium speed flash for Lora device error
    Serial.println(F("LoRa Error!"));
  }

  display_current_frequency();

  Serial.print(F("Cal Offset "));
  Serial.println(CalibrationOffset);

  lora_Print();
  print_SupplyVoltage();
  print_Temperature();
  GPS_Config_Error = false;                              //make sure GPS error flag is cleared

#ifndef DEBUGNoGPS
  GPS_On(GPSPowerControl);                               //GPS should have been on for a while by now, so this is just to start soft serial
  GPS_Setup();                                           //GPS should have had plenty of time to initialise by now

#ifdef UBLOX
  if (!GPS_CheckNavigation())                            //Check that UBLOX GPS is in Navigation model 6
  {
    Serial.println();
    GPS_Config_Error = true;
    setStatusByte(GPSError, 1);
  }
#endif

  if (GPS_Config_Error)
  {
    Serial.println(F("Warning GPS Error !"));
    Serial.println();
    led_Flash(100, 25);                                     //long very rapid flash for GPS error
  }
  else
  {
    Serial.println(F("Check Tone"));
    lora_Setup();                                                      //resets then sets up LoRa device
    lora_DirectSetup();                                                //set for direct mode
    lora_SetFreq(TrackerMode_Frequency, CalibrationOffset);
    lora_Tone(1000, 3000, 5);                            //Transmit an FM tone, 1000hz, 3000ms, 5dBm
  }

  setStatusByte(NoGPSTestMode, 0);

  while (!gpsWaitFix(5, DontSwitch, LeaveOn))             //while there is no fix
  {

    led_Flash(2, 50);                                     //two short LED flashes to indicate GPS waiting for fix

  }
#endif

  lora_Tone(500, 500, 2);                                  //Transmit an FM tone, 500hz, 500ms, 2dBm
  digitalWrite(LED1, LOW);

#ifdef DEBUGNoGPS
  Serial.println();
  Serial.println(F("No GPS Test Mode"));
  setStatusByte(NoGPSTestMode, 1);
  Serial.println();
#endif

}






