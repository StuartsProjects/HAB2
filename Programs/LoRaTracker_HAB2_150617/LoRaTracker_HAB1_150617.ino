//LoRaTracker_HAB1_150617.ino
#define programname "LoRaTracker_HAB1_150617"
#define programversion "V1.0"
#define dateproduced "15/06/17"
#define aurthorname "Stuart Robinson"

#include <Arduino.h>

#include "LoRaTracker_Settings_HAB.h"
#include "Program_Definitions.h"

/*
**************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 15/06/17

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  Location payload is constructed thus;

  PayloadID,Sequence,Time,Lat,Lon,Alt,Sats,SupplyVolts,Temperature,Resets,Config0byte,StatusByte,RunmAhr,hertzoffset,GPSFixTime,Checksum
   0           1      2   3   4   5    6      7            8         9        10         11        12       13         14       15

  To Do:


**************************************************************************************************
*/

char ramc_RemoteControlNode;
char ramc_ThisNode;

int ramc_CalibrationOffset;
unsigned long ramc_TrackerMode_Frequency;
unsigned long ramc_SearchMode_Frequency;
unsigned long ramc_CommandMode_Frequency;
byte ramc_TrackerMode_Bandwidth;
byte ramc_TrackerMode_SpreadingFactor;
byte ramc_TrackerMode_CodeRate;
byte ramc_TrackerMode_Power;
byte TRStatus;                                      //used to store current status flag bits

byte ramc_SearchMode_Bandwidth;
byte ramc_SearchMode_SpreadingFactor;
byte ramc_SearchMode_CodeRate;
byte ramc_SearchMode_Power;

byte ramc_CommandMode_Bandwidth;
byte ramc_CommandMode_SpreadingFactor;
byte ramc_CommandMode_CodeRate;
byte ramc_CommandMode_Power;

byte ramc_Current_TXconfig1;                        //sets the config of whats transmitted etc
byte ramc_Cmd_WaitSecs;
byte stripvalue;
byte sats;                                          //either sats in view from GPGSV or sats from NMEA fix sentence
int internal_temperature;


unsigned int ramc_Sleepsecs;                        //seconds for sleep at end of TX routine
unsigned int ramc_WaitGPSFixSeconds;                //in flight mode, default time to wait for a fix
unsigned int ramc_FSKRTTYbaudDelay;                 //dealy used in FSKRTTY routine to give chosen baud rate


unsigned long UPTime = 0;
unsigned long UPStart = 0;
unsigned long mASecs = 0;                           //running total of mAseconds used
unsigned long SleepmAsecs = 0;

float Fence_Check_Lon;                              //used for fence check

float TRLat;                                        //tracker transmitter co-ordinates
float TRLon;
unsigned int TRAlt;

//byte ramc_ControlledNode;
byte ramc_promiscuous_Mode;
byte ramc_FSKRTTYRegshift;
byte ramc_FSKRTTYpips;
unsigned int ramc_FSKRTTYleadin;
byte ramc_key0;
byte ramc_key1;
byte ramc_key2;
byte ramc_key3;
byte keypress;


//*************************************************************************************************


#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <Flash.h>

#include <TinyGPS++.h>
TinyGPSPlus gps;//create the TinyGPS++ object
TinyGPSCustom GNGGAFIXQ(gps, "GNGGA", 5);             //custom sentences used to detect possible switch to GLONASS mode
//TinyGPSCustom SATSINVIEW(gps, "GPGSV", 3);          //custom sentences used to take detect possible switch to GLONASS mode


#ifdef USE_SERIALGPS
#include "SerialGPS2.h"                               //uses the RAM efficient version of SerialGPS
#endif

#ifdef USE_I2CGPS
#include "I2CGPS.h"
#endif


#ifdef USE_SPI_FRAM_MEMORY1
#include "SPIFRAM_Memory1.h"
#endif

#ifdef USE_I2C_FRAM_MEMORY1
#include "I2CFRAM_Memory1.h"
#endif


#ifdef USE_EEPROM_MEMORY
#include "EEPROM_Memory.h"
#endif

#include "Voltage_Temperature.h"

#include <LowPower.h>
#include "LoRa3.h"
#include "FSK_RTTY2.h"
#include "Binary2.h"



void loop()
{

  UPStart = millis();                                    //set the start time for UPtime
  lora_TXTime = 0;
  lora_RXTime = 0;
  UPTime = 0;
  GPSFixTime = 0;
  internal_temperature = read_Temperature();             //read temp just after sleep, when CPU is closest to ambient

  wait_Command();

  if (readConfigByte(DozeEnable))
  {
    //tracker has doze made enabled, so just doze while and do not send trancmissions
    Serial.print(F("Doze "));
    updatemAUsed();
    printTimes();
    sleepSecs(DozeSleepSecs);
    return;
  }


  //if the tracker is in Doze mode we dont want to turn on the GPS


#ifndef DebugNoGPS
  GPSonTime = millis();
  gpsWaitFix(ramc_WaitGPSFixSeconds, SwitchOn, LeaveOn);
#endif

#ifdef DebugNoGPS
  TRLat = TestLatitude;
  TRLon = TestLongitude;
  TRAlt = TestAltitude;
#endif


  if (readConfigByte(CheckFence) && (!doFenceCheck()))      //if fence check is enabled and tracker is outside fence
  {
    action_outside_fence();
  }
  else                                                      //either fence check is disabled or tracker is within it
  {
    if (readConfigByte(TXEnable))                           //is TX enabled ?
    {
      do_Transmissions();                                   //yes, so do transmissions
    }
    else
    {
      Serial.println(F("Transmit disabled"));
      inside_fence_no_transmit();                           //no, TX is disabled
    }
  }

  updatemAUsed();
  printTimes();
  digitalWrite(lora_NReset, LOW);                           //take reset line low, makes sure LoRa device is off
  sleepSecs(Sleepsecs);

}

void wait_Command()
{
  byte index;
  pulseWDI();
  lora_Setup();                                              //resets then sets up LoRa device
  Setup_LoRaCommandMode();                                   //commands can be sent in any mode, make sure this is sent using the right frequency etc
  send_Command(ClearToSendCommand);                          //indicate ready for command

  lora_RXPacketType = 0;                                     //use as flag to tell if anything received during listen
  Listen(Cmd_WaitSecs);                                      //wait for command packet

  if (lora_RXPacketType > 0)
  {
    do
    {
      //there was activity during previous listen
      Serial.println(F("Listen Extended mode  "));
      lora_RXPacketType = 0;

      for (index = 1; index <= 6; index++)
      {
        Setup_LoRaCommandMode();                              //commands can be sent in any mode, make sure this is sent using the right frequency etc
        send_Command(ClearToSendCommand);
        Listen(Cmd_WaitSecs);
      }

    }  while (lora_RXPacketType > 0);                         //wait until the extended listen exits with no packet received

  }
  else
  {
    Serial.println(F("Nothing heard !"));
  }
}


void do_Transmissions()
{
  //this is where all the transmisions get sent
  byte index, Count;


  pulseWDI();
  lora_Setup();                                                      //resets then sets up LoRa device

  Setup_LoRaTrackerMode();

  incMemoryULong(addr_SequenceNum);                                  //increment sequence number
  Count = buildHABPacket(lora_TXBUFF);
  stripvalue = readConfigByte(AddressStrip);
  printPayload(Count);
  digitalWrite(LED1, HIGH);
  lora_Send(0, Count, HABPacket, Broadcast, ramc_ThisNode, 10, lora_Power, stripvalue);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  digitalWrite(LED1, LOW);
  Serial.println();

  sleepSecs(delayforRelaysecs);                                      //wait for relay to operate

  if (readConfigByte(SearchEnable))
  {
    Setup_LoRaSearchMode();                                           //setup is here so that any mode can be used to TX binary packet
    send_LocationBinary(TRLat, TRLon, TRAlt);
  }

  sleepSecs(1);

  if (readConfigByte(FSKRTTYEnable))
  {

#ifdef debug
    Serial.println(F("FSK RTTY Enabled ("));
    Serial.print(ramc_FSKRTTYbaudDelay);
    Serial.println(F(")"));
#endif

    start_FSKRTTY(FSKRTTYRegshift);
    for (index = 0; index <= Count; index++)
    {
      Send_FSKRTTY(lora_TXBUFF[index]);
    }
    Send_FSKRTTY(13);                                          //finish RTTY with carriage return
    Send_FSKRTTY(10);                                          //and line feed
    digitalWrite(LED1, LOW);                                   //make sure LED off
    Serial.println();
    lora_TXOFF();                                              //to ensure TXTime updated correctly
  }

  sleepSecs(6);                                                //allow time for receiver AFSK upload

  lora_TXOFF();                                                //to ensure low power mode in sleep

}


byte buildHABPacket(byte *lora_TXBUFF)
{
  //build the long tracker payload
  unsigned int index, j, volts, resets, sequence;
  unsigned int CRC;
  unsigned int runmAhr;
  byte Count;
  char LatArray[10], LonArray[10];
  char node[2];
  unsigned long fixtime;

  sequence = Memory_ReadULong(addr_SequenceNum);                //sequence number is kept in non-volatile memory so it survives resets
  resets = Memory_ReadULong(addr_ResetCount);                   //reset count is kept in non-volatile memory so it survives resets

  runmAhr = (mASecs / 3600);

  volts = read_SupplyVoltage();

  fixtime = (GPSFixTime / 1000);                               //make sure float is int
  sats = gps.satellites.value();
  dtostrf(TRLat, 7, 5, LatArray);                              //format is dtostrf(FLOAT,WIDTH,PRECISION,BUFFER);
  dtostrf(TRLon, 7, 5, LonArray);                              //converts float to character array

  memset(lora_TXBUFF, 0, sizeof(lora_TXBUFF));                 //clear array

  node[0] = ramc_ThisNode;
  node[1] = 0;

  snprintf(lora_TXBUFF,
           Output_len_max,
           "$$$$%s,%d,%02d:%02d:%02d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
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
           ramc_Current_TXconfig1,
           TRStatus,
           runmAhr,
           ramc_CalibrationOffset,
           fixtime
          );

  Count = strlen(lora_TXBUFF);                    //how long is the array ?

  CRC = 0xffff;                                   //start value for CRC16

  for (index = 4; index < Count; index++)         //element 4 is first character after $$$$ at start
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



void send_LocationBinary(float Lat, float Lon, unsigned int Alt)
{
  Serial.println(F("Send Location Binary Packet"));
  Serial.flush();

  Write_Float(0, Lat, lora_TXBUFF);
  Write_Float(4, Lon, lora_TXBUFF);
  Write_Uint(8, Alt, lora_TXBUFF);
  Write_Byte(10, TRStatus, lora_TXBUFF);

  digitalWrite(LED1, HIGH);
  lora_Send(0, 10, LocationBinaryPacket, Broadcast, ramc_ThisNode, 10, lora_Power, stripvalue);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  digitalWrite(LED1, LOW);
}


void inside_fence_no_transmit()
{
  Setup_LoRaCommandMode();                                   //commands can be sent in any mode, make sure this is sent using the right frequency etc
  Listen(ramc_Cmd_WaitSecs);                                 //wait for command packet}
}


void action_outside_fence()
{
  sleepSecs(outside_fence_Sleep_seconds);                    //goto sleep for 30 Minutes
  lora_Setup();
  Setup_LoRaCommandMode();
  Listen(Cmd_WaitSecs);
  Listen(Cmd_WaitSecs);
}


byte doFenceCheck()                                         //checks to see if GPS location is within an Western and Eastern limit
{
  //there has been a fix, so check fence limits
  Serial.println(F("Fence Check"));
  Fence_Check_Lon = gps.location.lng();
  if ((Fence_Check_Lon > west_fence) && (Fence_Check_Lon < east_fence))   //approximate to the limits for region 1 ISM band
  {
    Serial.println(F("Inside Fence"));
    return 1;                                               //within the fence

  }
  Serial.println(F("Outside Fence"));
  return 0;                                                 //outside the fence
}


void printPayload(byte lCount)
{
  byte index;
  for (index = 0; index <= lCount; index++)
  {
    Serial.write(lora_TXBUFF[index]);
  }
}


void updatemAUsed()
{
  addmASecs(TXmA, lora_TXTime);                             //add TXTime current consumed current to total
  addmASecs(RXmA, lora_RXTime);                             //add RXTime current consumed current to total

  if (readConfigByte(GPSPowerSave))
  {
    addmASecs(GPSmA, GPSFixTime);                           //add GPS consumed current to total only if power save enabled
  }

  UPTime = (millis() - UPStart);
  addmASecs(runmA, UPTime);                                 //add run current consumed current to total
  addmASecs(SleepmA, SleepmAsecs);
  Memory_WriteULong(addr_mASecs, mASecs);                   //save mASecs count in memory
  SleepmAsecs = 0;
}


void addmASecs(byte lmAamp, unsigned long lmSecs)
{
  //adds to the running total of mASecs, i.e 10mA for 2000mS = 20mAmS
  //for a long unsigned int max Count = 4294967296 or 4294967 mAMins or 71582 maHr
  unsigned long i, j;
  i = (lmSecs / 1000);                                      //convert the mS time into Seconds
  j = i * lmAamp;                                           //calculate the mASecs
  mASecs = mASecs + j;
  Serial.print(F("mASecs "));
  Serial.println(mASecs);
}


void printTimes()
{
  //print the times used to calculate mAhr used
  Serial.print(F("TXTime "));
  Serial.print(lora_TXTime);
  Serial.println(F("mS"));
  Serial.print(F("RXTime "));
  Serial.print(lora_RXTime);
  Serial.println(F("mS"));
  Serial.print(F("UPTime "));
  Serial.print((millis() - UPStart));
  Serial.println(F("mS"));
  Serial.print(F("mASecs "));
  Serial.println(mASecs);
  Serial.print(F("mAHour "));
  Serial.println((mASecs) / 3600);
}


void Listen(unsigned int seconds)
{
  //listen (seconds) for an incoming packet using the current frequency and LoRa modem settings
  unsigned long tilltime;
  tilltime = (millis() + (seconds * 1000));
  Serial.print(F("Listen "));
  Serial.println(seconds);

  lora_RXONLoRa();

  while (millis() < tilltime)
  {
    checkForPacket();
  }
  lora_RXOFF();                                     //as we have finished listening
}


byte checkForPacket()
{
  //check LoRa device to see if a packet has arrived
  byte lora_LRegData;
  byte lora_Ltemp;

  lora_Ltemp = lora_readRXready();

  if (lora_Ltemp > 0)
  {
    if (lora_Ltemp == 64)
    {
      //packet has arrived
      lora_ReadPacket();
      Serial.print(F("RX "));
      Serial.write(lora_RXPacketType);
      Serial.write(lora_RXDestination);
      Serial.write(lora_RXSource);
      Serial.println();
    }
    else
    {
      //packet arrived with error
      Serial.println(F("Packet Error"));
      lora_RXOFF;
      lora_RXONLoRa();
    }


    if (promiscuous_Mode)                           //can we accept packet from any source
    {
      processPacket();
      lora_RXONLoRa();
    }


    if (!promiscuous_Mode)                          //can we only accepts packet from known node
    {
      if (lora_RXSource == ramc_RemoteControlNode)
      {
        processPacket();
        lora_RXONLoRa();                            //ready for next and clear flags
      }
      else
      {
        Serial.println(F("Rejected Packet"));
        lora_RXOFF;
        Setup_LoRaCommandMode();
        send_Command(NACK);
        lora_RXONLoRa();
      }
    }

  }

}


void processPacket()
{
  //we have a packet so lets decide what to do with it
  byte i, j, ptr;

  if (lora_RXPacketType == Test)
  {
    if (lora_RXBUFF[0] == '0')
    {
      Serial.println(F("LoRa Packet Test Request"));
      delay(inter_Packet_delay);
      Setup_LoRaCommandMode();
      send_Command(ACK);
      delay(inter_Packet_delay);
      sendTest();
    }

    if (lora_RXBUFF[0] == '1')
    {
      Serial.println(F("FSK RTTY Test Request"));
      delay(inter_Packet_delay);
      Setup_LoRaCommandMode();
      send_Command(ACK);
      delay(inter_Packet_delay);
      Send_FSKRTTYTest();
    }

  }

  if (lora_RXPacketType == LinkReport)
  {
    send_Command(ACK);
    delay(inter_Packet_delay);
    Serial.println(F("Request Link Report"));
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(Info);
  }


  if (lora_RXPacketType == Config0)                   //is it a change config byte request ?
  {
    Serial.println(F("Program Configbyte"));

    i = ((lora_RXBUFF[0] - 48));                      //config byte requests come in as ASCCI, '1' for 1 etc
    j = ((lora_RXBUFF[1] - 48));
    setconfigByte(i, j);
    lora_RXBuffPrint(0);                              //print packet contents as ASCII
    Serial.println();
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);                                //send the ack

    //its possible there has been a command to swap GPS out of power on\off mode
    //therefore the GPS needs to be woken up and setup for cyclic power save as the
    //software backup command will cancel power save
    //gpsOn(GPSPowerControl);
    //setPowersaveCyclic();
  }

  if (lora_RXPacketType == ResetTracker)              //is it a reset ?
  {
    Serial.println(F("Request reset"));
    lora_RXBuffPrint(0);                              //print packet contents as ASCII
    Serial.println();

    if ( isKeyValid() )
    {
      Serial.println(F("Request Valid"));
      delay(inter_Packet_delay);
      Setup_LoRaCommandMode();
      send_Command(ACK);
      Serial.flush();
      sleepSecs(2);
      softReset();
    }
    else
    {
      Serial.println(F("Invalid Request"));
      delay(inter_Packet_delay);
      //delay(500);
      Setup_LoRaCommandMode();
      send_Command(NACK);
    }
  }

  if (lora_RXPacketType == WritePacketMemory)
  {
    Serial.println(F("Write to Memory"));
    writePacketMemory();
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);
  }

  if (lora_RXPacketType == INCFreq)
  {
    Serial.println(F("Increase Calibration Offset 1KHZ"));
    ramc_CalibrationOffset = ramc_CalibrationOffset + 1000;
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);
    Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);
    printRAMFrequencies();
  }

  if (lora_RXPacketType == DECFreq)
  {
    Serial.println(F("Decrease Calibration Offset 1KHZ"));
    ramc_CalibrationOffset = ramc_CalibrationOffset - 1000;
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);
    Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);
    printRAMFrequencies();
  }

  if (lora_RXPacketType == Bind)
  {

    if (isKeyValid())                                           //only accept bind request when key is valid
    {

      ptr = 4;                                                  //bind packet has 4 bytes of key
      Serial.println(F("Bind Received"));

      for (i = addr_StartConfigData; i <= addr_EndConfigData; i++)
      {
        j = lora_RXBUFF[ptr++];
        Memory_WriteByte(i, j);
      }

      readSettingsMemory();

#ifdef DEBUG
      printMemoryFrequencies();
      printRAMFrequencies();
      Config_Memory_Print();
      i = Memory_CRC(addr_StartConfigData, addr_EndConfigData);
      Serial.print(F("Config CRC "));
      Serial.println(i, HEX);
#endif
      delay(inter_Packet_delay);
      send_Command(ACK);
    }
    else
    {
      Serial.println(F("Key not valid - Bind Ingored"));
    }

  }
}


void printMemoryFrequencies()
{
  //a useful check to see if memory is configured correctly
  unsigned long tempULong;
  tempULong = Memory_ReadULong(addr_TrackerMode_Frequency);
  Serial.print(F("Memory Tracker Frequency "));
  Serial.println(tempULong);

  tempULong = Memory_ReadULong(addr_SearchMode_Frequency);
  Serial.print(F("Memory Search Frequency "));
  Serial.println(tempULong);

  tempULong = Memory_ReadULong(addr_CommandMode_Frequency);
  Serial.print(F("Memory Command Frequency "));
  Serial.println(tempULong);
}


void printRAMFrequencies()
{
  Serial.print(F("RAM Tracker Frequency "));
  Serial.println(ramc_TrackerMode_Frequency);

  Serial.print(F("RAM Search Frequency "));
  Serial.println(ramc_SearchMode_Frequency);

  Serial.print(F("RAM Command Frequency "));
  Serial.println(ramc_CommandMode_Frequency);

  Serial.print(F("RAM Calibration Offset "));
  Serial.println(ramc_CalibrationOffset);
}


boolean isKeyValid()
{
  if ( (lora_RXBUFF[0] == key0) && (lora_RXBUFF[1] == key1)  && (lora_RXBUFF[2] == key2)  && (lora_RXBUFF[3] == key3) )
  {
    return true;
  }
  else
  {
    return false;
  }
}


void softReset()
{
  asm volatile ("  jmp 0");
}


void send_Command(char cmd)
{
  Serial.print(F("Send Command "));
  Serial.write(cmd);
  Serial.println();
  Write_Byte(0, lora_PacketSNR, lora_TXBUFF);
  Write_Byte(1, lora_PacketRSSI, lora_TXBUFF);
  Write_Uint(2, read_SupplyVoltage(), lora_TXBUFF);
  Write_Byte(4, TRStatus, lora_TXBUFF);
  digitalWrite(LED1, HIGH);
  lora_Send(0, 4, cmd, Broadcast, ramc_ThisNode, 10, lora_Power, 0);
  digitalWrite(LED1, LOW);
}


void sendTest()
{
  byte power;
  for (power = 10; power >= 2; power--)
  {
    Setup_LoRaTrackerMode();
    lora_TXBUFF[0] = '0';
    lora_TXBUFF[1] = power + 48;

    if (power == 10)
    {
      lora_TXBUFF[0] = '1';
      lora_TXBUFF[1] = '0';
    }

    Serial.print(F("Send "));
    lora_TXBuffPrint(0);
    Serial.write(lora_TXBUFF[1]);
    Serial.println();

    lora_Send(0, 1, Test, Broadcast, ramc_ThisNode, 10, power, 0);     //send the test packet
    sleepSecs(2);
  }
}


void Send_FSKRTTYTest()
{
  byte power;

  start_FSKRTTY(FSKRTTYRegshift);
  Send_FSKRTTY(13);
  Send_FSKRTTY(10);

  for (power = 10; power >= 2; power--)
  {
    lora_TXONDirect(power);
    delay(200);
    Send_FSKRTTY(' ');


    if (power == 10)
    {
      Send_FSKRTTY('1');
      Send_FSKRTTY('0');
    }
    else
    {
      Send_FSKRTTY('0');
      Send_FSKRTTY(power + 48);
    }
    delay(200);
  }
  Send_FSKRTTY(13);
  Send_FSKRTTY(10);
  lora_Setup();
}


void sleepSecs(int LNumberSleeps)
{
  int i;

  Serial.print(F("zzz "));
  Serial.println(LNumberSleeps);

  Serial.flush();                                      //let print complete
  digitalWrite(lora_NSS, HIGH);                        //ensure LoRa Device is off

  for (i = 1; i <= LNumberSleeps; i++)
  {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);    //sleep 1 second
    pulseWDI();
  }
  SleepmAsecs = SleepmAsecs + LNumberSleeps;
}


void incMemoryULong(unsigned long laddress)
{
  unsigned long val;
  val = Memory_ReadULong(laddress);
  val++;
  Memory_WriteULong(laddress, val);
}


byte readConfigByte(byte bitnum)
{
  return bitRead(ramc_Current_TXconfig1, bitnum);
}


void setconfigByte(byte bitnum, byte bitval)
{
  //program the config byte

  if (bitval == 0)
  {
    bitClear(ramc_Current_TXconfig1, bitnum);
  }
  else
  {
    bitSet(ramc_Current_TXconfig1, bitnum);
  }
  Memory_WriteByte(addr_Default_config1, ramc_Current_TXconfig1);
  Serial.print(F("Set bit "));
  Serial.print(bitnum);
  Serial.print(F(" to "));
  Serial.print(bitval);
  Serial.print(F(" "));
  Serial.println(ramc_Current_TXconfig1, BIN);
}


void setstatusByte(byte bitnum, byte bitval)
{
  //program the status byte
#ifdef Debug
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

  if (bitval == 0)
  {
    bitClear(TRStatus, bitnum);
  }
  else
  {
    bitSet(TRStatus, bitnum);
  }
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

  if (StartState == 1)
  {
    gpsOn(UseGPSPowerControl);
  }
  else
  {
    gpsOn(DoNotUseGPSPowerControl);
  }

  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.println(F(" Seconds"));

  currentmillis = millis();
  millistowait = waitSecs * 1000;
  endwaitmS = currentmillis + millistowait;

  while (millis() < endwaitmS)
  {
    getprocessGPSchar();

    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    {
      Serial.println(F("GPS Fix !"));
      TRLat = gps.location.lat();
      TRLon = gps.location.lng();
      TRAlt = (unsigned int) gps.altitude.meters();

      if (readConfigByte(GPSPowerSave))
      {
        gpsOff(UseGPSPowerControl);
      }
      else
      {
        gpsOff(DoNotUseGPSPowerControl);
      }

      setstatusByte(GPSFix, 1);
      pulseWDI();
      return true;
    }


  }

  //if here then there has been no fix and a timeout
  setstatusByte(GPSFix, 0);                       //set status bit to flag no fix
  Serial.print(F("No GPS Fix"));

#ifdef UBLOX
  if (GNGGAFIXQ.age() < 2000)                     //check to see if GLONASS has gone active
  {
    Serial.println(F("GLONASS Active"));
    Setup_LoRaCommandMode();
    send_Command(GLONASSDetected);
    setconfigByte(GLONASSisoutput, 1);
    config_UBLOXGPS();
    setup_GPSPowerSave();                         //set power save mode
    sleepSecs(1);
    setup_GPSCyclic();
    sleepSecs(1);
  }
  else
  {
    setstatusByte(GLONASSisoutput, 0);            //GLONASS not detected
  }
#endif

  //the possibility of Glonass mode has now been dealt with,

  if (LeaveState == 0)
  {
    //no fix but gpsWaitFix called with gpspower to be turned off on exit
    gpsOff(UseGPSPowerControl);
  }
  else
  {
    //no fix but gpsWaitFix called with gpspower to be left on at exit
    gpsOff(DoNotUseGPSPowerControl);
  }

  pulseWDI();
  return false;
}


void writeArrayMemory(unsigned char *Config, int Length, unsigned int Mem_address)
{
  //saves the array at the address passed into Memory, used to save flight ID into Memory
  int i = 0;
  byte j;
  Serial.print(F("Saving Array "));

  while (i < Length)
  {
    j = Config[i++];
    Memory_WriteByte(Mem_address++, j);
    Serial.write(j);
  }
  Serial.println();
}


void read_array_Memory(unsigned char *Config, unsigned int Length, unsigned int Mem_address)
{
  //reads from EEPROM into the the array at the address passed, used to recover flight ID
  int i = 0;
  byte j;

  while (i < Length)
  {
    j = Memory_ReadByte(Mem_address);
    Config[i++] = j;
    Serial.write(j);
  }
  Serial.println();
}


void readSettingsDefaults()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this program uses constants in RAM copied from Memory in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver.
  Serial.println(F("Configuring Settings from Defaults"));
  ramc_TrackerMode_Frequency = TrackerMode_Frequency;
  ramc_TrackerMode_Bandwidth = TrackerMode_Bandwidth;
  ramc_TrackerMode_SpreadingFactor = TrackerMode_SpreadingFactor;
  ramc_TrackerMode_CodeRate = TrackerMode_CodeRate;
  ramc_TrackerMode_Power = TrackerMode_Power;

  ramc_SearchMode_Frequency = SearchMode_Frequency;
  ramc_SearchMode_Bandwidth = SearchMode_Bandwidth;
  ramc_SearchMode_SpreadingFactor = SearchMode_SpreadingFactor;
  ramc_SearchMode_CodeRate = SearchMode_CodeRate;
  ramc_SearchMode_Power = SearchMode_Power;

  ramc_CommandMode_Frequency = CommandMode_Frequency;
  ramc_CommandMode_Bandwidth = CommandMode_Bandwidth;
  ramc_CommandMode_SpreadingFactor = CommandMode_SpreadingFactor;
  ramc_CommandMode_CodeRate = CommandMode_CodeRate;
  ramc_CommandMode_Power = CommandMode_Power;

  ramc_ThisNode = ThisNode;
  ramc_RemoteControlNode = RemoteControlNode;

  ramc_Current_TXconfig1 = Default_config1;
  ramc_Cmd_WaitSecs = Cmd_WaitSecs;
  ramc_WaitGPSFixSeconds = WaitGPSFixSeconds;
  ramc_Sleepsecs = Sleepsecs;
  ramc_promiscuous_Mode = promiscuous_Mode;

  ramc_FSKRTTYbaudDelay = FSKRTTYbaudDelay;
  ramc_FSKRTTYRegshift = FSKRTTYRegshift;
  ramc_FSKRTTYpips = FSKRTTYpips;
  ramc_FSKRTTYleadin = FSKRTTYleadin;

  ramc_key0 = key0;
  ramc_key1 = key1;
  ramc_key2 = key2;
  ramc_key3 = key3;

  ramc_CalibrationOffset = CalibrationOffset;
}


void readSettingsMemory()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this program uses constants in RAM copied from Memory (EEPROM) in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver.
  Serial.println(F("Configuring Settings from Memory"));

  ramc_TrackerMode_Frequency = Memory_ReadULong(addr_TrackerMode_Frequency);
  ramc_TrackerMode_Bandwidth = Memory_ReadByte(addr_TrackerMode_Bandwidth);
  ramc_TrackerMode_SpreadingFactor = Memory_ReadByte(addr_TrackerMode_SpreadingFactor);
  ramc_TrackerMode_CodeRate = Memory_ReadByte(addr_TrackerMode_CodeRate);
  ramc_TrackerMode_Power = Memory_ReadByte(addr_TrackerMode_Power);

  ramc_SearchMode_Frequency = Memory_ReadULong(addr_SearchMode_Frequency);
  ramc_SearchMode_Bandwidth = Memory_ReadByte(addr_SearchMode_Bandwidth);
  ramc_SearchMode_SpreadingFactor = Memory_ReadByte(addr_SearchMode_SpreadingFactor);
  ramc_SearchMode_CodeRate = Memory_ReadByte(addr_SearchMode_CodeRate);
  ramc_SearchMode_Power = Memory_ReadByte(addr_SearchMode_Power);

  ramc_CommandMode_Frequency = Memory_ReadULong(addr_CommandMode_Frequency);
  ramc_CommandMode_Bandwidth = Memory_ReadByte(addr_CommandMode_Bandwidth);
  ramc_CommandMode_SpreadingFactor = Memory_ReadByte(addr_CommandMode_SpreadingFactor);
  ramc_CommandMode_CodeRate = Memory_ReadByte(addr_CommandMode_CodeRate);
  ramc_CommandMode_Power = Memory_ReadByte(addr_CommandMode_Power);

  ramc_ThisNode = Memory_ReadByte(addr_ThisNode);;
  ramc_RemoteControlNode = Memory_ReadByte(addr_RemoteControlNode);;

  ramc_Current_TXconfig1 = Memory_ReadByte(addr_Default_config1);
  ramc_Cmd_WaitSecs = Memory_ReadByte(addr_Cmd_WaitSecs);
  ramc_WaitGPSFixSeconds = Memory_ReadUInt(addr_WaitGPSFixSeconds);
  ramc_Sleepsecs = Memory_ReadUInt(addr_Sleepsecs);
  ramc_promiscuous_Mode = Memory_ReadByte(addr_promiscuous_Mode);

  ramc_FSKRTTYbaudDelay = Memory_ReadUInt(addr_FSKRTTYbaudDelay);
  ramc_FSKRTTYRegshift = Memory_ReadByte(addr_FSKRTTYRegshift);
  ramc_FSKRTTYpips = Memory_ReadByte(addr_FSKRTTYpips);
  ramc_FSKRTTYleadin = Memory_ReadUInt(addr_FSKRTTYleadin);
  ramc_key0 = Memory_ReadByte(addr_key0);
  ramc_key1 = Memory_ReadByte(addr_key1);
  ramc_key2 = Memory_ReadByte(addr_key2);
  ramc_key3 = Memory_ReadByte(addr_key3);

  ramc_CalibrationOffset = Memory_ReadInt(addr_CalibrationOffset);

  readIDMemory();
}


void writeSettingsMemory()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this program uses constants in RAM copied from Memory (EEPROM or FRAM)

  Serial.println(F("Writing RAM Settings to Memory"));
  Memory_Set(addr_StartConfigData, addr_EndConfigData, 0);          //fill config area with 0
  writeIDMemory();

  Memory_WriteULong(addr_TrackerMode_Frequency, ramc_TrackerMode_Frequency);
  Memory_WriteByte(addr_TrackerMode_Bandwidth, ramc_TrackerMode_Bandwidth);
  Memory_WriteByte(addr_TrackerMode_SpreadingFactor, ramc_TrackerMode_SpreadingFactor);
  Memory_WriteByte(addr_TrackerMode_CodeRate, ramc_TrackerMode_CodeRate);
  Memory_WriteByte(addr_TrackerMode_Power, ramc_TrackerMode_Power);

  Memory_WriteULong(addr_SearchMode_Frequency, ramc_SearchMode_Frequency);
  Memory_WriteByte(addr_SearchMode_Bandwidth, ramc_SearchMode_Bandwidth);
  Memory_WriteByte(addr_SearchMode_SpreadingFactor, ramc_SearchMode_SpreadingFactor);
  Memory_WriteByte(addr_SearchMode_CodeRate, ramc_SearchMode_CodeRate);
  Memory_WriteByte(addr_SearchMode_Power, ramc_SearchMode_Power);

  Memory_WriteULong(addr_CommandMode_Frequency, ramc_CommandMode_Frequency);
  Memory_WriteByte(addr_CommandMode_Bandwidth, ramc_CommandMode_Bandwidth);
  Memory_WriteByte(addr_CommandMode_SpreadingFactor, ramc_CommandMode_SpreadingFactor);
  Memory_WriteByte(addr_CommandMode_CodeRate, ramc_CommandMode_CodeRate);
  Memory_WriteByte(addr_CommandMode_Power, ramc_CommandMode_Power);

  Memory_WriteByte(addr_Default_config1, ramc_Current_TXconfig1);
  Memory_WriteByte(addr_RemoteControlNode, ramc_RemoteControlNode);

  Memory_WriteByte(addr_ThisNode, ramc_ThisNode);
  Memory_WriteByte(addr_RemoteControlNode, ramc_RemoteControlNode);

  Memory_WriteByte(addr_Default_config1, ramc_Current_TXconfig1);
  Memory_WriteByte(addr_Cmd_WaitSecs, ramc_Cmd_WaitSecs);
  Memory_WriteUInt(addr_WaitGPSFixSeconds, ramc_WaitGPSFixSeconds);
  Memory_WriteUInt(addr_Sleepsecs, ramc_Sleepsecs);
  Memory_WriteByte(addr_promiscuous_Mode, ramc_promiscuous_Mode);

  Memory_WriteUInt(addr_FSKRTTYbaudDelay, ramc_FSKRTTYbaudDelay);
  Memory_WriteByte(addr_FSKRTTYRegshift, ramc_FSKRTTYRegshift);
  Memory_WriteByte(addr_FSKRTTYpips, ramc_FSKRTTYpips);
  Memory_WriteUInt(addr_FSKRTTYleadin, ramc_FSKRTTYleadin);
  Memory_WriteByte(addr_key0, ramc_key0);
  Memory_WriteByte(addr_key1, ramc_key1);
  Memory_WriteByte(addr_key2, ramc_key2);
  Memory_WriteByte(addr_key3, ramc_key3);

  Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);
}


void writeIDMemory()
{
  unsigned int i, addr;
  byte j;
  j = sizeof(Flight_ID);
  j--;
  addr = addr_FlightID;
  for (i == 0; i <= j; i++)
  {
    Memory_WriteByte(addr, Flight_ID[i]);
    addr++;
  }
}

void readIDMemory()
{
  unsigned int addr;
  byte i;
  byte j = 0;

  addr = addr_FlightID;
  do
  {
    i = Memory_ReadByte(addr);
    Flight_ID[j++] = i;
    addr++;
  }
  while (i != 0) ;
}


void saveKeyin_buffer()
{
  lora_TXBUFF[0] = key0;       //key used in sme packets to reduce chances of a change being applied by accident
  lora_TXBUFF[1] = key1;
  lora_TXBUFF[2] = key2;
  lora_TXBUFF[3] = key3;
}


void printMemoryChange(byte number)
{
  byte index, j;
  Serial.print(F("Memory Change"));
  for (index = 0; index <= number; index++)
  {
    j = lora_RXBUFF[index];
    Serial.print(F(" "));
    Serial.print(j, HEX);
  }
  Serial.println();

}


byte writePacketMemory()
{
  //there is an incoming packet which is a request to write bytes to Memory.
  //the effect is to change stored program definitions and constants
  byte i, j, k = 0;
  byte ptr;
  byte low, high;
  unsigned int addr_Memory;
  float tempfloat;

  //packet format is key0, key1, key2, key3, number of bytes to write, address to write to, bytes to write
  //terminate list with 0 bytes to write.

  if (isKeyValid())
  {
    Serial.print(F("Not Valid Request"));
    return 0;
  }

  i = lora_RXPacketL - 4;                      //end of packet will be length - 1 for 0 offset and -3 for adddress bytes

  printMemoryChange(i);

  ptr = 4;

  j = lora_RXBUFF[ptr++];

  addr_Memory = Read_Uint(5, lora_RXBUFF);     //read address for frequency offset into buffer

  ptr++;
  ptr++;

  Serial.println(F("Writing updates "));

  for (i = 1; i <= j; i++)
  {
    Memory_WriteByte(addr_Memory, lora_RXBUFF[ptr]);
    k = lora_RXBUFF[ptr];
    Serial.print(k, HEX);
    Serial.print(F(" "));
    addr_Memory++;
    ptr++;
  }
  Serial.println(F("  Changes Written"));
  readSettingsMemory();
  Setup_LoRaTrackerMode();                     //dummy change so we can see if offset chnages
}


void  do_ClearSavedData()
{
  Serial.println(F("Clear Data"));
  Memory_WriteULong(addr_ResetCount, 0);
  Memory_WriteULong(addr_SequenceNum, 0);
  Memory_WriteULong(addr_mASecs, 0);
}


void sendTrackerBind()
{
  int i, j;
  byte msb_CRC, lsb_CRC;
  unsigned int bindCRC;

  //byte plength = 3;                          //the bind packet data will start at byte 4
  saveKeyin_buffer();                          //loads key in bytes 0,1,2,3 of TX buffer

  lora_TXEnd = 4;                              //this is where the bind data starts

  for (i = addr_StartBindData; i <= addr_EndBindData; i++)
  {
    j =  Memory_ReadByte(i);
    lora_TXBUFF[lora_TXEnd++] = j;
  }

  bindCRC = Memory_CRC(addr_StartBindData, addr_EndBindData);
  msb_CRC = highByte(bindCRC);
  lsb_CRC = lowByte(bindCRC);
  lora_TXBUFF[lora_TXEnd++] = lsb_CRC;
  lora_TXBUFF[lora_TXEnd] = msb_CRC;

  Serial.print(F("Sending Bind Packet Length "));
  Serial.println(lora_TXEnd + 4);               //allow for 3 addressing bytes in length, plus 1 for packet starting at [0]

  lora_Send(0, lora_TXEnd, Bind, ramc_RemoteControlNode, ramc_ThisNode, 10, BindMode_Power, 0);
  Serial.print(F("Bind Data CRC 0x"));
  Serial.println(bindCRC, HEX);
}



void setPowersaveCyclic()
{
#ifndef DebugNoGPS
  setup_GPSPowerSave();                         //set power save mode
  delay(1000);
  setup_GPSCyclic();
  delay(1000);
#endif

#ifdef DebugNoGPS
  Serial.println(F("NoGPS is set"));
#endif
}


void printNodes()
{ Serial.print(F("This Node "));
  Serial.print(ramc_ThisNode);
  Serial.print(F("  RemoteControlNode "));
  Serial.println(ramc_RemoteControlNode);
  Serial.println();
}


void Setup_LoRaTrackerMode()
{
  lora_SetFreq(ramc_TrackerMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_TrackerMode_Bandwidth, ramc_TrackerMode_SpreadingFactor, ramc_TrackerMode_CodeRate, Explicit);  //Setup the LoRa modem parameters for tracker mode
  lora_Power = ramc_TrackerMode_Power;
}


void Setup_LoRaSearchMode()
{
  lora_SetFreq(ramc_SearchMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_SearchMode_Bandwidth, ramc_SearchMode_SpreadingFactor, ramc_SearchMode_CodeRate, Explicit);  //Setup the LoRa modem parameters for search mode
}

void Setup_LoRaCommandMode()
{
  lora_SetFreq(ramc_CommandMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_CommandMode_Bandwidth, ramc_CommandMode_SpreadingFactor, ramc_CommandMode_CodeRate, Explicit);  //Setup the LoRa modem parameters for command mode
  lora_Power = ramc_CommandMode_Power;
}


void Setup_LoRaBindMode()
{
  lora_SetFreq(BindMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(BindMode_Bandwidth, BindMode_SpreadingFactor, BindMode_CodeRate, Explicit); //Setup the LoRa modem parameters for bind mode
  lora_Power = BindMode_Power;
}

void display_current_frequency()
{
  float freq_temp;
  freq_temp = lora_GetFreq();
  Serial.print(F("Current Frequency "));
  Serial.print(freq_temp, 3);
  Serial.println(F("MHz"));
}



void led_FlashStart()
{
  //flash LED to show tracker is alive
  byte index;

  for (index = 0; index <= 4; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(150);
    digitalWrite(LED1, LOW);
    delay(150);
  }
}


void setup()
{
  unsigned long int i;
  unsigned int tempint;
  byte val;

  pinMode(LED1, OUTPUT);                  //for PCB LED
  pinMode(WDI, OUTPUT);                   //for Watchdog pulse input

  led_FlashStart();

  Serial.begin(38400);                    //Setup Serial console ouput

  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));

#ifdef DEBUG
  Serial.print(F("Default_config1 "));
  Serial.println(Default_config1, BIN);
#endif

  pinMode(GPSPOWER, OUTPUT);              //in case power switching components are fitted
  gpsOn(UseGPSPowerControl);              //this will power the GPSon

#ifdef USE_SERIALGPS
  GPSserial.end();                        //but we dont want soft serial running for now, it interferes with the LoRa device
#endif

  pinMode(lora_NReset, OUTPUT);           //LoRa device reset line
  digitalWrite(lora_NReset, HIGH);

  pinMode (lora_NSS, OUTPUT);             //set the slave select pin as an output:
  digitalWrite(lora_NSS, HIGH);

  SPI.begin();                            //initialize SPI
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  Serial.print(F("Resets "));
  i = Memory_ReadULong(addr_ResetCount);
  Serial.println(i);
  incMemoryULong(addr_ResetCount);

#ifdef Debug
  mASecs = Memory_ReadULong(addr_mASecs);
  Serial.print(F("Stored mASecs "));
  Serial.println(mASecs);
#endif

#ifdef ClearSavedData
  do_ClearSavedData();
#endif

  printTimes();


#ifdef ConfigureDefaults
  readSettingsDefaults();
  writeSettingsMemory();
#endif

#ifdef ConfigureFromMemory
  readSettingsMemory();
#endif

#ifdef write_CalibrationOffset
  Memory_WriteInt(addr_CalibrationOffset, CalibrationOffset);
#endif

  ramc_ThisNode = ThisNode;
  printNodes();


  lora_Setup();
  Setup_LoRaTrackerMode();

  display_current_frequency();
  ramc_CalibrationOffset = Memory_ReadInt(addr_CalibrationOffset);  //get calibration offset for this tracker
  Serial.print(F("Calibration Offset "));
  Serial.println(ramc_CalibrationOffset);


#ifdef DEBUG
  lora_Print();
#endif


#ifdef CalibrateTone
  if (readConfigByte(TXEnable))   //is TX enabled ?
  {
    Serial.println(F("Transmit Tone"));
    lora_Tone(1000, 500, 5);                          //Transmit an FM tone, 1000hz, 500ms, 5dBm
  }
#endif

  Serial.println();
  print_SupplyVoltage();
  print_Temperature();
  Serial.println();

  tempint = read_SupplyVoltage();                      //get supply mV
  Write_Uint(0, tempint, lora_TXBUFF);                 //write to first two bytes of buffer
  Write_Byte(2, ramc_Current_TXconfig1, lora_TXBUFF);  //add the current config byte

  Setup_LoRaTrackerMode();
  send_Command(PowerUp);                                //send power up command, includes supply mV and config, on tracker settings
  sleepSecs(1);

#ifdef SendBind
  Setup_LoRaBindMode();

#ifdef DEBUG
  Config_Memory_Print();
#endif

  if (readConfigByte(TXEnable))   //is TX enabled ?
  {
    sendTrackerBind();
  }
#endif

#ifndef DebugNoGPS
  gpsOn(GPSPowerControl);                               //GPS should have been on for a while by now, so this is just to start soft serial
  gpsSetup();                                           //GPS should have had plenty of time to initialise by now
#endif

  Setup_LoRaTrackerMode();


#ifndef DebugNoGPS
  digitalWrite(LED1, HIGH);
  setstatusByte(NoGPSTestMode, 0);
  GPSonTime = millis();
  while (!gpsWaitFix(5, DontSwitch, LeaveOn)) //while there is no fix
  {
    lora_TXBUFF[0] = 0;
    if (readConfigByte(TXEnable))                      //is TX enabled ?
    {
      send_Command(NoFix);
      delay(inter_Packet_delay);
    }

#ifdef Debug
    i = (millis() - GPSonTime) / 1000;
    Serial.print(F("GPS On Time "));
    Serial.print(i);
    Serial.println(F(" Secs"));
#endif

  }
  addmASecs(GPSmA, GPSFixTime);                        //add GPS consumed current to total
#endif
  lora_Tone(500, 500, 2);                              //Transmit an FM tone, 500hz, 500ms, 2dBm
  digitalWrite(LED1, LOW);
  sleepSecs(2);                                        //wait for GPS to shut down

#ifdef DebugNoGPS
  setstatusByte(NoGPSTestMode, 1);
#endif

  setconfigByte(DozeEnable, Disabled);                 //ensure Doze mode disabled at reset

}

