//**************************************************************************************************
// Note:
//
// Make changes to this Program file at your peril
//
// Configuration changes should be made in the HAB2_Settings file not here !
//
//**************************************************************************************************

#define programname "LoRaTracker_HAB2_Bare_Bones_200518"
#define aurthorname "Stuart Robinson"

#include <Arduino.h>
#include <avr/pgmspace.h>

#include "LoRaTracker_HAB2_Settings.h"
#include Board_Definition
#include "Program_Definitions.h"

/*
**************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  Location payload is constructed thus;

  PayloadID,Sequence,Time,Lat,Lon,Alt,Sats,SupplyVolts,Temperature,Resets,StatusByte,GPSFixTime,Checksum
  0           1      2   3   4   5    6      7            8         9        10          11          12        
  
  To Do:

  Changes: 
  
  220518 - Develop a version of the HAB2 software reduced to a minium
  220518 - Size with SoftwareSerial GPS 29996/1407, I2C GPS 28402/1290
  220518 - Leave with I2C GPS
  220518 - Remove battery use calculation 27440/1286
  220518 - Remove CPU_VoltageRead option
  220518 - Remove memory storage of config, use system defaults 
  220518 - Remove receiver options, fence check 20942/1091
  220518 - Switch to UBLOX_I2CGPS2.h 20388/1009
  220518 - Remove unecessary commands and byte definitions 20050/949
  220518 - Allow removal of detecting GNSS mode, should not be needed for latest TinyGPS++ 19928/905

**************************************************************************************************
*/

byte TRStatus = 0;                                  //used to store current status flag bits

byte stripvalue;
byte sats;                                          //either sats in view from GPGSV or sats from NMEA fix sentence
int temperature;

float TRLat;                                        //tracker transmitter co-ordinates
float TRLon;
unsigned int TRAlt;

byte keypress;

unsigned long GPSonTime;
unsigned long GPSFixTime;
boolean GPS_Config_Error;

#include Board_Definition                            //include previously defined board file
#include Memory_Library                              //include previously defined Memory Library

#include <SPI.h>
#include <LowPower.h>                                //https://github.com/rocketscream/Low-Power

#include <TinyGPS++.h>                               //http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                     //create the TinyGPS++ object

#ifdef Checkfor_GNNS_Mode
TinyGPSCustom GNGGAFIXQ(gps, "GNGGA", 5);            //custom sentences used to detect possible switch to GNSS mode
#endif


#ifdef USE_SOFTSERIAL_GPS
#include <NeoSWSerial.h>                             //https://github.com/SlashDevin/NeoSWSerial  
NeoSWSerial GPSserial(GPSRX, GPSTX);                 //this library is more relaible at GPS init than software serial
#endif

#include GPS_Library                                 //include previously defined GPS Library 

#include "Voltage_Temperature.h"
#include "LoRa3.h"
#include "FSK_RTTY2.h"
#include "Binary2.h"


void loop()
{
  Serial.println();
  
  GPSFixTime = 0;
  temperature = (int) read_Temperature();           //read temp just after sleep, when CPU is closest to ambient
  GPSonTime = millis();

#ifndef DEBUGNoGPS
  gpsWaitFix(WaitGPSFixSeconds, SwitchOn, LeaveOn);
#endif

#ifdef Use_Test_Location
  TRLat = TestLatitude;
  TRLon = TestLongitude;
  TRAlt = TestAltitude;
#endif

  do_Transmissions();                                               //do the transmissions

  digitalWrite(lora_NSS, HIGH);                                     //take NSS line high, makes sure LoRa device is off
  sleepSecs(Loop_Sleepsecs);                                        //this sleep is used to set overall transmission cycle time

}


void do_Transmissions()
{
  //this is where all the transmisions get sent
  byte index, Count;

  pulseWDI();
  lora_Setup();                                                      //resets then sets up LoRa device

  Setup_LoRaTrackerMode();

  incMemoryULong(addr_SequenceNum);                                  //increment sequence number
  Count = buildHABPacket();
  stripvalue = readConfigByte(AddressStrip);
  Serial.println(F("HAB Pkt"));
  printPayload(Count);
  Serial.println();
  digitalWrite(LED1, HIGH);
  lora_Send(0, Count, HABPacket, Broadcast, ThisNode, 10, lora_Power, stripvalue);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  digitalWrite(LED1, LOW);

  sleepSecs(delayforAFSKuploadSecs);                                 //allow time for receiver AFSK upload

  sleepSecs(delayforRelaysecs);                                      //wait for relay to operate

  if (readConfigByte(FSKRTTYEnable))
  {

    Serial.println(F("FSKRTTY"));

    lora_DirectSetup();                                               //set for direct mode
    lora_SetFreq(TrackerMode_Frequency, CalibrationOffset);
    Start_FSKRTTY(FSKRTTYRegshift, FSKRTTYleadin, FSKRTTYpips);

    for (index = 1; index <= sync_chars; index++)
    {
      Send_FSKRTTY('$', FSKRTTYbaudDelay);
    }

    for (index = 0; index <= Count; index++)
    {
      Send_FSKRTTY(lora_TXBUFF[index], FSKRTTYbaudDelay);
    }
    Send_FSKRTTY(13, FSKRTTYbaudDelay);                        //finish RTTY with carriage return
    Send_FSKRTTY(10, FSKRTTYbaudDelay);                        //and line feed
    digitalWrite(LED1, LOW);                                   //make sure LED off
    lora_TXOFF();                                              //to ensure TXTime updated correctly
  }

  if (readConfigByte(SearchEnable))
  {
    Setup_LoRaSearchMode();                                    //setup is here so that any mode can be used to TX binary packet
    send_LocationBinary(TRLat, TRLon, TRAlt);
  }
}


byte buildHABPacket()                                          //expects a char buffer, so this routine will not work without the -permissive setting
{
  //build the long tracker payload
  unsigned int index, j, CRC, resets;
  int volts;
  byte Count,len;
  char LatArray[12], LonArray[12];
  unsigned long sequence;

  sequence = Memory_ReadULong(addr_SequenceNum);               //sequence number is kept in non-volatile memory so it survives resets
  resets =  Memory_ReadUInt(addr_ResetCount);                  //reset count is kept in non-volatile memory so it survives resets

  volts = read_SupplyVoltage();

  if (!readConfigByte(GPSHotFix))
  {
    GPSFixTime = 0;                                            //if GPS power save is off (no GPSHotFix), ensure GPS fix time is set to zero
  }

  sats = gps.satellites.value();
  dtostrf(TRLat, 7, 5, LatArray);                              //format is dtostrf(FLOAT,WIDTH,PRECISION,BUFFER);
  dtostrf(TRLon, 7, 5, LonArray);                              //converts float to character array
  len = sizeof(lora_TXBUFF);
  memset(lora_TXBUFF, 0, len);                                 //clear array to 0s

  Count = snprintf((char*) lora_TXBUFF,
                   Output_len_max,
                   "$$$$%s,%lu,%02d:%02d:%02d,%s,%s,%d,%d,%d,%d,%d,%d,%lu",
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
                   temperature,
                   resets,
                   TRStatus,
                   GPSFixTime
                  );

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


char Hex(byte lchar)
{
  //used in CRC calculation in buildHABPacket
  char Table[] = "0123456789ABCDEF";
  return Table[lchar];
}



void send_LocationBinary(float Lat, float Lon, unsigned int Alt)
{
  Write_Float(0, Lat, lora_TXBUFF);
  Write_Float(4, Lon, lora_TXBUFF);
  Write_Int(8, Alt, lora_TXBUFF);
  Write_Byte(10, TRStatus, lora_TXBUFF);

  digitalWrite(LED1, HIGH);
  Serial.println(F("Binary Location"));
  lora_Send(0, 10, LocationBinaryPacket, Broadcast, ThisNode, 10, lora_Power, 0);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  digitalWrite(LED1, LOW);
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
#ifdef USING_SERIALGPS
  GPSserial.end();                                     //we dont want GPS input interfering with sleep, make sure its off
#endif
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

#ifdef DEBUG
  if (bitval)
  {
    Serial.print(F("Set Status Bit "));
  }
  else
  {
    Serial.print(F("Clear Status Bit "));
  }

  Serial.println(bitnum);
#endif
}


byte readConfigByte(byte bitnum)
{
  return bitRead(Default_config1, bitnum);
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
    GPS_On(DoGPSPowerSwitch);
    GPSonTime = millis();
  }
  else
  {
    GPS_On(NoGPSPowerSwitch);
  }

#ifdef Check_GPS_Navigation_Model_OK
  Serial.println(F("Check GPSNavigation Model"));
  if (!GPS_CheckNavigation())
  {
    //something wrong with GPS, navigation mode not set so reconfigure the GPS
    Serial.println(F("GPS Config Error !!!!"));
    GPS_Setup();
  }
#endif


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

      //Altitude is used as an unsigned integer, so that the binary payload is as short as possible.
      //However gps.altitude.meters(); can return a negative value which converts to
      //65535 - Altitude, which we dont want. So we will assume any value over 60,000M is zero

      if (TRAlt > 60000)
      {
        TRAlt = 0;
      }

      if (readConfigByte(GPSHotFix))
      {
        GPS_Off(DoGPSPowerSwitch);
        GPSFixTime = (millis() - GPSonTime);
        Serial.print(F("FixTime "));
        Serial.print(GPSFixTime);
        Serial.println(F("mS"));
      }
      else
      {
        GPS_Off(NoGPSPowerSwitch);
      }

      setStatusByte(GPSFix, 1);
      pulseWDI();
      return true;
    }

#ifdef UBLOX
#ifdef Checkfor_GNNS_Mode
    if (GNGGAFIXQ.age() < 2000)                     //check to see if GNSS mode has gone active
    {
      Serial.println(F("GNSS !"));
      setStatusByte(GLONASSisoutput, 1);
      GPS_SetGPMode();
      GPS_SetCyclicMode();
      sleepSecs(1);
    }
    else
    {
      setStatusByte(GLONASSisoutput, 0);            //GNSS mode not detected
    }
#endif    
#endif



  }

  //if here then there has been no fix and a timeout
  setStatusByte(GPSFix, 0);                       //set status bit to flag no fix
  Serial.println(F("No Fix"));

  if (LeaveState == 0)
  {
    //no fix and gpsWaitFix called with gpspower to be turned off on exit
    GPS_Off(DoGPSPowerSwitch);
  }
  else
  {
    //no fix but gpsWaitFix called with gpspower to be left on at exit
    GPS_Off(NoGPSPowerSwitch);
  }

  pulseWDI();
  return false;
}



void printNodes()
{
  Serial.print(F("ThisNode "));
  Serial.print(ThisNode);
  Serial.println();
}


void Setup_LoRaTrackerMode()
{
  lora_SetFreq(TrackerMode_Frequency, CalibrationOffset);
  lora_SetModem2(TrackerMode_Bandwidth, TrackerMode_SpreadingFactor, TrackerMode_CodeRate, Explicit);  //Setup the LoRa modem parameters for tracker mode
  lora_Power = TrackerMode_Power;
}


void Setup_LoRaSearchMode()
{
  lora_SetFreq(SearchMode_Frequency, CalibrationOffset);
  lora_SetModem2(SearchMode_Bandwidth, SearchMode_SpreadingFactor, SearchMode_CodeRate, Explicit);  //Setup the LoRa modem parameters for search mode
  lora_Power = SearchMode_Power;
}


void send_Command(char cmd)
{
  unsigned int volts;
  volts = read_SupplyVoltage();
  Serial.print(F("Send Cmd "));
  Serial.write(cmd);
  Serial.println();
  Write_Byte(0, lora_PacketSNR, lora_TXBUFF);                         //so that receiver alwsys knows last received SNR
  Write_Byte(1, lora_PacketRSSI, lora_TXBUFF);                        //so that receiver alwsys knows last received RSSI
  Write_UInt(2, volts, lora_TXBUFF);
  Write_Byte(4, TRStatus, lora_TXBUFF);
  digitalWrite(LED1, HIGH);
  lora_Send(0, 4, cmd, Broadcast, ThisNode, 10, lora_Power, 0);
  digitalWrite(LED1, LOW);
}


void display_current_frequency()
{
  float freq_temp = lora_GetFreq();
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
  Serial.println(F("Clear Memory"));
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

  Memory_Start();

#ifdef ClearAllMemory
  Clear_All_Memory();
#endif

  Serial.println(F(programname));
  Serial.println(F(aurthorname));

  pinMode(GPSPOWER, OUTPUT);              //in case power switching components are fitted
  GPS_On(DoGPSPowerSwitch);               //this will power the GPSon
  GPSonTime = millis();

#ifdef USING_SERIALGPS
  GPSserial.end();                        //but we dont want soft serial running for now, it interferes with the LoRa device
#endif

  pinMode(lora_NReset, OUTPUT);           //LoRa device reset line
  digitalWrite(lora_NReset, HIGH);

  pinMode (lora_NSS, OUTPUT);             //set the slave select pin as an output:
  digitalWrite(lora_NSS, HIGH);

  SPI.begin();                            //initialize SPI
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));


#ifdef ClearSavedData
  do_ClearSavedData();
#endif

  Serial.print(F("Default_Config1 "));
  Serial.println(Default_config1, BIN);

  j = Memory_ReadUInt(addr_ResetCount);
  j++;
  Memory_WriteUInt(addr_ResetCount, j);
  Serial.print(F("Resets "));
  Serial.println(j);
  Serial.print(F("Sequence "));
  i = Memory_ReadULong(addr_SequenceNum);
  Serial.println(i);

#ifdef DEBUG
  printNodes();
#endif

  lora_Setup();

  if (!lora_CheckDevice())
  {
    led_Flash(100, 50);                                                //long medium speed flash for Lora device error
    Serial.println(F("LoRa Error!"));
  }

#ifdef DEBUG
  display_current_frequency();
#endif

  Serial.print(F("Cal Offset "));
  Serial.println(CalibrationOffset);

#ifdef DEBUG
  lora_Print();
#endif

  Serial.println();
  print_SupplyVoltage();
  print_Temperature();
  Serial.println();

  j = read_SupplyVoltage();                             //get supply mV

  Setup_LoRaTrackerMode();
  send_Command(PowerUp);                                //send power up command, includes supply mV and config, on tracker settings
  sleepSecs(1);

  Setup_LoRaTrackerMode();                              //so that check tone is at correct frequency

  GPS_Config_Error = false;                             //make sure GPS error flag is cleared

#ifndef DEBUGNoGPS
  GPS_On(DoGPSPowerSwitch);                             //GPS should have been on for a while by now, so this is just to start soft serial
  GPSonTime = millis();
  GPS_Setup();                                          //GPS should have had plenty of time to initialise by now

#ifdef UBLOX
  if (!GPS_CheckNavigation())                           //Check that UBLOX GPS is in Navigation model 6
  {
    Serial.println();
    GPS_Config_Error = true;
    setStatusByte(GPSError, 1);
    setStatusByte(UBLOXDynamicModel6Set, 0);
  }
  else
  {
    setStatusByte(UBLOXDynamicModel6Set, 1);
  }
#endif

  if (GPS_Config_Error)
  {
    Serial.println(F("GPS Error !"));
    Serial.println();
    send_Command(NoGPS);                                    //make sure receiver knows about GPS error
    led_Flash(100, 25);                                     //long very rapid flash for GPS error
  }
  else
  {

#ifdef CheckTone
    digitalWrite(LED1, HIGH);
    Serial.println(F("Check Tone"));                      //check tone indicates navigation model 6 set (if checktone enabled!)
    lora_Tone(1000, 3000, 2);                             //Transmit an FM tone, 1000hz, 3000ms, 2dBm
    digitalWrite(LED1, LOW);
#endif
  }

  
  setStatusByte(NoGPSTestMode, 0);
  GPSonTime = millis();
  while (!gpsWaitFix(5, DontSwitch, LeaveOn))           //wait for the initial GPS fix, this could take a while, leave GPS powered on
  {

  led_Flash(2, 50);                                     //two short LED flashes to indicate GPS waiting for fix

#ifdef DEBUG
    i = (millis() - GPSonTime) / 1000;
    Serial.print(F("GPS OnTime "));
    Serial.print(i);
    Serial.println(F(" Secs"));
#endif
  }
  
#endif

#ifndef DEBUGNoGPS
  GPS_On(DoGPSPowerSwitch);                                                 
  GPS_SetCyclicMode();                                     //set this regardless of whether hot fix mode is enabled
#endif

  lora_Tone(500, 500, 2);                                  //Transmit an FM tone, 500hz, 500ms, 2dBm
  digitalWrite(LED1, LOW);
  sleepSecs(2);                                            //wait for GPS to shut down

#ifdef DEBUGNoGPS
  setStatusByte(NoGPSTestMode, 1);
#endif

}

