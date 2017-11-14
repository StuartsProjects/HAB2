//**************************************************************************************************
// Note:
//
// Make changes to this Program file at your peril
//
// Configuration changes should be made in the HAB2_Settings file not here !
//
//**************************************************************************************************

#define programname "LoRaTracker_GPSBeacon_Beta_101017"
#define aurthorname "Stuart Robinson"

#include <Arduino.h>
#include <avr/pgmspace.h>

#include "LoRaTracker_GPSBeacon_Settings.h"
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

  This is a simple GPS locator beacon, the GPS is read and the current location sent the specified period. The beacon can 
  optionally transmit a ready to send command packet, so it would be possible the remote control the beacon.

  Changes:

**************************************************************************************************
*/

char ramc_RemoteControlNode;
char ramc_ThisNode;

int ramc_CalibrationOffset;

unsigned long ramc_TrackerMode_Frequency;           //frequencies, and other parameters, are copied from memory into RAM.
unsigned long ramc_CommandMode_Frequency;           //copied into memory so that they survive reset.

uint8_t ramc_TrackerMode_Bandwidth;
uint8_t  ramc_TrackerMode_SpreadingFactor;
uint8_t  ramc_TrackerMode_CodeRate;
uint8_t  ramc_TrackerMode_Power;
uint8_t  TRStatus = 0;                              //used to store current status flag bits

uint8_t  ramc_CommandMode_Bandwidth;
uint8_t  ramc_CommandMode_SpreadingFactor;
uint8_t  ramc_CommandMode_CodeRate;
uint8_t  ramc_CommandMode_Power;

uint8_t  ramc_Current_TXconfig1;                        //sets the config of whats transmitted etc
uint8_t  ramc_Cmd_WaitSecs;
uint8_t  stripvalue;
uint8_t  sats;                                          //either sats in view from GPGSV or sats from NMEA fix sentence
int internal_temperature;

boolean GPS_Config_Error;
uint16_t ramc_Sleepsecs;                        //seconds for sleep at end of TX routine
uint16_t ramc_WaitGPSFixSeconds;                //in flight mode, default time to wait for a fix

float TRLat;                                        //tracker transmitter co-ordinates
float TRLon;
uint16_t TRAlt;

uint8_t  ramc_promiscuous_Mode;
uint8_t  ramc_key0;
uint8_t  ramc_key1;
uint8_t  ramc_key2;
uint8_t  ramc_key3;
uint8_t  keypress;


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
#include "LoRa4.h"
#include "Binary2.h"



void loop()
{
  Serial.println();
  Serial.println();


#ifndef DEBUGNoGPS
  gpsWaitFix(ramc_WaitGPSFixSeconds, SwitchOn, LeaveOn);
#endif

#ifdef Use_Test_Location
  TRLat = TestLatitude;
  TRLon = TestLongitude;
  TRAlt = TestAltitude;
#endif

  do_Transmissions();  

  sleepSecs(1);  

  #ifdef Accept_Commands
  wait_Command();                                     //wait for incoming command
  #endif
  
  digitalWrite(lora_NSS, HIGH);                             //take NSS line high, makes sure LoRa device is off
  sleepSecs(ramc_Sleepsecs);                                //this sleep is used to set overall transmission cycle time
}


void wait_Command()
{
  uint8_t  index;
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
      lora_RXPacketType = 0;

      for (index = 1; index <= Command_Loops; index++)
      {
        Setup_LoRaCommandMode();                              //commands can be sent in any mode, make sure this is sent using the right frequency etc
        send_Command(ClearToSendCommand);
        Listen(Cmd_WaitSecs);
      }

    }  while (lora_RXPacketType > 0);                         //wait until the extended listen exits with no packet received

  }
  else
  {
    Serial.println(F("No RX"));
  }
}


void do_Transmissions()
{
  //this is where all the transmisions get sent

  pulseWDI();
  lora_Setup();                                                      //resets then sets up LoRa device
  Setup_LoRaTrackerMode();
  send_LocationBinary(TRLat, TRLon, TRAlt);
}


void send_LocationBinary(float Lat, float Lon, uint16_t Alt)
{
  Write_Float(0, Lat, lora_TXBUFF);
  Write_Float(4, Lon, lora_TXBUFF);
  Write_Int(8, Alt, lora_TXBUFF);
  Write_Byte(10, TRStatus, lora_TXBUFF);

  digitalWrite(LED1, HIGH);
  Serial.println(F("Send Binary Location"));
  lora_Send(0, 10, LocationBinaryPacket, Broadcast, ramc_ThisNode, 10, lora_Power, 0);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  digitalWrite(LED1, LOW);
}



void Listen(uint16_t seconds)
{
  //listen (in seconds) for an incoming packet using the current frequency and LoRa modem settings
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


void checkForPacket()
{
  //check LoRa device to see if a command packet has arrived
  uint8_t  lora_Ltemp;

  lora_Ltemp = lora_readRXready();

  if (lora_Ltemp > 0)
  {

    Serial.print(F("RX "));

    if (lora_Ltemp == 64)
    {
      //packet has arrived
      lora_ReadPacket();

      Serial.write(lora_RXPacketType);
      Serial.write(lora_RXDestination);
      Serial.write(lora_RXSource);
      Serial.println();
    }
    else
    {
      //packet arrived with error
      Serial.println(F("Error"));
      lora_RXOFF();
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
        Serial.println(F("Rejected"));
        lora_RXOFF();
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
  uint8_t  i, j;

  if (lora_RXPacketType == Test)
  {
    if (lora_RXBUFF[0] == '0')
    {
      Serial.println(F("Pkt Test"));
      delay(inter_Packet_delay);
      Setup_LoRaCommandMode();
      send_Command(ACK);
      delay(inter_Packet_delay);
      sendTest();
    }
  }

  if (lora_RXPacketType == LinkReport)
  {
    send_Command(ACK);
    delay(inter_Packet_delay);
    Serial.println(F("Link Report"));
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(Info);
  }


  if (lora_RXPacketType == Config0)                   //is it a change config uint8_t  request ?
  {
    Serial.println(F("Prog Cfgbyte"));

    i = ((lora_RXBUFF[0] - 48));                      //config uint8_t  requests come in as ASCCI, '1' for 1 etc
    j = ((lora_RXBUFF[1] - 48));
    setConfigByte(i, j);
    lora_RXBuffPrint(0);                              //print packet contents as ASCII
    Serial.println();
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);                                //send the ack

  }

  if (lora_RXPacketType == ResetTracker)              //is it a reset ?
  {
    Serial.println(F("Reset ?"));
    lora_RXBuffPrint(0);                              //print packet contents as ASCII
    Serial.println();

    if ( isKeyValid() )
    {
      Serial.println(F("Valid"));
      delay(inter_Packet_delay);
      Setup_LoRaCommandMode();
      send_Command(ACK);
      Serial.flush();
      sleepSecs(2);
      softReset();
    }
    else
    {
      Serial.println(F("Invalid"));
      delay(inter_Packet_delay);
      Setup_LoRaCommandMode();
      send_Command(NACK);
    }
  }

  if (lora_RXPacketType == WritePacketMemory)
  {
    Serial.println(F("Write Memory"));
    writePacketMemory();
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);
  }

  if (lora_RXPacketType == INCFreq)
  {
    Serial.println(F("IncOffset 1KHZ"));
    ramc_CalibrationOffset = ramc_CalibrationOffset + 1000;
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);
    Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);
  }

  if (lora_RXPacketType == DECFreq)
  {
    Serial.println(F("DecOffset 1KHZ"));
    ramc_CalibrationOffset = ramc_CalibrationOffset - 1000;
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);
    Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);
   }
   
}




void printMemoryFrequencies()
{
  //a useful check to see if memory is configured correctly
  unsigned long tempULong;
  tempULong = Memory_ReadULong(addr_TrackerMode_Frequency);
  Serial.print(F("Memory Tracker Freq "));
  Serial.println(tempULong);

  tempULong = Memory_ReadULong(addr_CommandMode_Frequency);
  Serial.print(F("Memory Command Freq "));
  Serial.println(tempULong);
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
  uint16_t volts;
  volts = read_SupplyVoltage();
  Serial.print(F("Send Cmd "));
  Serial.write(cmd);
  Serial.println();
  Write_Byte(0, lora_PacketSNR, lora_TXBUFF);                         //so that receiver alwsys knows last received SNR
  Write_Byte(1, lora_PacketRSSI, lora_TXBUFF);                        //so that receiver alwsys knows last received RSSI
  Write_UInt(2, volts, lora_TXBUFF);
  Write_Byte(4, TRStatus, lora_TXBUFF);
  digitalWrite(LED1, HIGH);
  lora_Send(0, 4, cmd, Broadcast, ramc_ThisNode, 10, lora_Power, 0);
  digitalWrite(LED1, LOW);
}


void sendTest()
{
  uint8_t  power;
  for (power = 10; power >= 2; power--)
  {
    Setup_LoRaTrackerMode();
    lora_TXBUFF[0] = '0';
    lora_TXBUFF[1] = power + 48;
    lora_TXEnd = 1;
    
    if (power == 10)
    {
      lora_TXBUFF[0] = '1';
      lora_TXBUFF[1] = '0';
      lora_TXEnd = 2;
    }

    Serial.print(F("Send "));
    Serial.print(power);
    Serial.println(F("dBm"));

    lora_Send(0, 1, Test, Broadcast, ramc_ThisNode, 10, power, 0);     //send the test packet
    sleepSecs(2);
  }
}

void sleepSecs(uint16_t LNumberSleeps)
{
  uint16_t i;

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


void incMemoryULong(uint16_t laddress)
{
  unsigned long val;
  val = Memory_ReadULong(laddress);
  val++;
  Memory_WriteULong(laddress, val);
}


uint8_t  readConfigByte(uint8_t  bitnum)
{
  return bitRead(ramc_Current_TXconfig1, bitnum);
}


void setConfigByte(uint8_t  bitnum, uint8_t  bitval)
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

}


void setStatusByte(uint8_t  bitnum, uint8_t  bitval)
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

}


void pulseWDI()
{
  //if the watchdog is fitted it needs a regular pulse top prevent reset
  //togle the WDI pin twice
  digitalWrite(WDI, !digitalRead(WDI));
  delayMicroseconds(1);
  digitalWrite(WDI, !digitalRead(WDI));
}


boolean gpsWaitFix(unsigned long waitSecs, uint8_t  StartState, uint8_t  LeaveState)
{
  //waits a specified number of seconds for a fix, returns true for good fix
  //StartState when set to 1 will turn GPS on at routine start
  //LeaveState when set to 0 will turn GPS off at routine end, used perhaps when there is no fix

  uint32_t endwaitmS, millistowait, currentmillis;
  pulseWDI();
  uint8_t  GPSByte;
  //ong temp;

  if (StartState == 1)
  {
    GPS_On(DoGPSPowerSwitch);
  }
  else
  {
    GPS_On(NoGPSPowerSwitch);
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
      GPSByte  = GPS_GetByte();
      if (GPSByte  != 0xFF)
      {
        gps.encode(GPSByte);
      }
    }
    while (GPSByte  != 0xFF);

    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    {
      Serial.println(F("GPS Fix"));
      TRLat = gps.location.lat();
      TRLon = gps.location.lng();
      TRAlt = (uint16_t) gps.altitude.meters();

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
    if (GNGGAFIXQ.age() < 2000)                     //check to see if GLONASS has gone active
    {
      Serial.println(F("GLONASS !"));
      setStatusByte(GLONASSisoutput, 1);
      GPS_SetGPMode();
      GPS_SetCyclicMode();
      sleepSecs(1);
    }
    else
    {
      setStatusByte(GLONASSisoutput, 0);            //GLONASS not detected
    }
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


void readSettingsDefaults()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this program uses constants in RAM copied from Memory (EEPROM or FRAM) in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver.
  Serial.println(F("Config Defaults"));
  ramc_TrackerMode_Frequency = TrackerMode_Frequency;
  ramc_TrackerMode_Bandwidth = TrackerMode_Bandwidth;
  ramc_TrackerMode_SpreadingFactor = TrackerMode_SpreadingFactor;
  ramc_TrackerMode_CodeRate = TrackerMode_CodeRate;
  ramc_TrackerMode_Power = TrackerMode_Power;

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
  ramc_Sleepsecs = Loop_Sleepsecs;
  ramc_promiscuous_Mode = promiscuous_Mode;

  ramc_key0 = key0;
  ramc_key1 = key1;
  ramc_key2 = key2;
  ramc_key3 = key3;

  ramc_CalibrationOffset = CalibrationOffset;
}


void readSettingsMemory()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this program uses constants in RAM copied from Memory (EEPROM or FRAM) in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver.
  Serial.println(F("Config from Memory"));

  ramc_TrackerMode_Frequency = Memory_ReadULong(addr_TrackerMode_Frequency);
  ramc_TrackerMode_Bandwidth = Memory_ReadByte(addr_TrackerMode_Bandwidth);
  ramc_TrackerMode_SpreadingFactor = Memory_ReadByte(addr_TrackerMode_SpreadingFactor);
  ramc_TrackerMode_CodeRate = Memory_ReadByte(addr_TrackerMode_CodeRate);
  ramc_TrackerMode_Power = Memory_ReadByte(addr_TrackerMode_Power);

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

  ramc_key0 = Memory_ReadByte(addr_key0);
  ramc_key1 = Memory_ReadByte(addr_key1);
  ramc_key2 = Memory_ReadByte(addr_key2);
  ramc_key3 = Memory_ReadByte(addr_key3);

  ramc_CalibrationOffset = Memory_ReadInt(addr_CalibrationOffset);

}


void writeSettingsMemory()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this program uses constants in RAM copied from Memory (EEPROM or FRAM)

  Serial.println(F("Write RAM Settings to Memory"));

  Memory_Set(addr_StartConfigData, addr_EndConfigData, 0);          //fill config area with 0

  Memory_WriteULong(addr_TrackerMode_Frequency, ramc_TrackerMode_Frequency);
  Memory_WriteByte(addr_TrackerMode_Bandwidth, ramc_TrackerMode_Bandwidth);
  Memory_WriteByte(addr_TrackerMode_SpreadingFactor, ramc_TrackerMode_SpreadingFactor);
  Memory_WriteByte(addr_TrackerMode_CodeRate, ramc_TrackerMode_CodeRate);
  Memory_WriteByte(addr_TrackerMode_Power, ramc_TrackerMode_Power);

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

  Memory_WriteByte(addr_key0, ramc_key0);
  Memory_WriteByte(addr_key1, ramc_key1);
  Memory_WriteByte(addr_key2, ramc_key2);
  Memory_WriteByte(addr_key3, ramc_key3);

  Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);
}


void saveKeyin_buffer()
{
  lora_TXBUFF[0] = key0;       //key used in sme packets to reduce chances of a change being applied by accident
  lora_TXBUFF[1] = key1;
  lora_TXBUFF[2] = key2;
  lora_TXBUFF[3] = key3;
}


void printMemoryChange(uint8_t  number)
{
  uint8_t  index, j;
  Serial.print(F("Memory Change"));
  for (index = 0; index <= number; index++)
  {
    j = lora_RXBUFF[index];
    Serial.print(F(" "));
    Serial.print(j, HEX);
  }
  Serial.println();
}


void writePacketMemory()
{
  //there is an incoming packet which is a request to write bytes to Memory.
  //the effect is to change stored program definitions and constants
  uint8_t  i, j, k, ptr;
  //uint8_t  i, j, k, ptr, low, high;
  uint16_t addr_Memory;
  //float tempfloat;

  //packet format is key0, key1, key2, key3, number of bytes to write, address to write to, bytes to write
  //terminate list with 0 bytes to write.

  if (isKeyValid())
  {
    Serial.print(F("Not Valid"));
    return;
  }

  i = lora_RXPacketL - 4;                      //end of packet will be length - 1 for 0 offset and -3 for adddress bytes

  printMemoryChange(i);

  ptr = 4;

  j = lora_RXBUFF[ptr++];

  addr_Memory = Read_Int(5, lora_RXBUFF);     //read address for frequency offset into buffer

  ptr++;
  ptr++;

  Serial.println(F("Write memory "));

  for (i = 1; i <= j; i++)
  {
    Memory_WriteByte(addr_Memory, lora_RXBUFF[ptr]);
    k = lora_RXBUFF[ptr];
    Serial.print(k, HEX);
    Serial.print(F(" "));
    addr_Memory++;
    ptr++;
  }
  readSettingsMemory();
  Setup_LoRaTrackerMode();                     //dummy change so we can see if offset chnages
}



void sendTrackerBind()
{
  uint16_t i, j;
  uint8_t  msb_CRC, lsb_CRC;
  uint16_t bindCRC;

  saveKeyin_buffer();                          //loads key in bytes 0,1,2,3 of TX buffer

  lora_TXEnd = 4;                              //this is where the bind data starts

  for (i = addr_StartBindData; i <= addr_EndBindData; i++)
  {
    j =  Memory_ReadByte(i);
    lora_TXBUFF[lora_TXEnd++] = j;
  }

  bindCRC = Print_CRC_Bind_Memory();
  msb_CRC = highByte(bindCRC);
  lsb_CRC = lowByte(bindCRC);
  lora_TXBUFF[lora_TXEnd++] = lsb_CRC;
  lora_TXBUFF[lora_TXEnd] = msb_CRC;

  Serial.print(F("Bind PacketLen "));
  Serial.println(lora_TXEnd + 4);               //allow for 3 addressing bytes in length, plus 1 for packet starting at [0]

  lora_Send(0, lora_TXEnd, Bind, ramc_RemoteControlNode, ramc_ThisNode, 10, BindMode_Power, 0);

}


void printNodes()
{
  Serial.print(F("ThisNode "));
  Serial.print(ramc_ThisNode);
  Serial.print(F("  RemoteNode "));
  Serial.println(ramc_RemoteControlNode);
  Serial.println();
}


void Setup_LoRaTrackerMode()
{
  lora_SetFreq(ramc_TrackerMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_TrackerMode_Bandwidth, ramc_TrackerMode_SpreadingFactor, ramc_TrackerMode_CodeRate, Explicit);  //Setup the LoRa modem parameters for tracker mode
  lora_Power = ramc_TrackerMode_Power;
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
  Serial.print(F("Frequency "));
  Serial.print(freq_temp, 3);
  Serial.println(F("MHz"));
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  //flash LED to show tracker is alive
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


//*******************************************************************************************************
// Memory Routines
//*******************************************************************************************************

void Clear_All_Memory()
{
  //clears the whole of memory, normally 1kbyte
  Serial.print(F("Clear All Memory"));
  Memory_Set(addr_StartMemory, addr_EndMemory, 0);
}


void Print_Config_Memory()
{
  //prints the memory used for storing configuration settings
  uint8_t  memory_LLoopv1;
  uint8_t  memory_LLoopv2;
  uint16_t memory_Laddr = 0;
  uint8_t  memory_Ldata;
  //uint16_t CRC;
  Serial.println(F("Config Memory"));
  Serial.print(F("Lcn    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();

  for (memory_LLoopv1 = 0; memory_LLoopv1 <= 15; memory_LLoopv1++)
  {
    Serial.print(F("0x"));
    Serial.print(memory_LLoopv1, HEX);                       //print the register number
    Serial.print(F("0  "));
    for (memory_LLoopv2 = 0; memory_LLoopv2 <= 15; memory_LLoopv2++)
    {
      memory_Ldata = Memory_ReadByte(memory_Laddr);
      if (memory_Ldata < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(memory_Ldata, HEX);                       //print the register number
      Serial.print(F(" "));
      memory_Laddr++;
    }
    Serial.println();
  }
}


void Print_CRC_Config_Memory()
{
  uint16_t returnedCRC = Memory_CRC(addr_StartConfigData, addr_EndConfigData);
  Serial.print(F("CRC_Config "));
  Serial.println(returnedCRC, HEX);
}


uint16_t Print_CRC_Bind_Memory()
{
  uint16_t returnedCRC = Memory_CRC(addr_StartBindData, addr_EndBindData);
  Serial.print(F("Local BindCRC "));
  Serial.println(returnedCRC, HEX);
  return returnedCRC;
}


void setup()
{
  uint16_t j;

  pinMode(LED1, OUTPUT);                  //for PCB LED
  pinMode(WDI, OUTPUT);                   //for Watchdog pulse input

  led_Flash(2, 500);

  Serial.begin(38400);                    //Setup Serial console ouput

#ifdef ClearAllMemory
  Clear_All_Memory();
#endif


  Serial.println(F(programname));
  Serial.println(F(aurthorname));

  pinMode(GPSPOWER, OUTPUT);              //in case power switching components are fitted
  GPS_On(DoGPSPowerSwitch);               //this will power the GPSon

#ifdef USING_SERIALGPS
  GPSserial.end();                        //but we dont want soft serial running for now, it interferes with the LoRa device
#endif

  pinMode(lora_NReset, OUTPUT);           //LoRa device reset line
  digitalWrite(lora_NReset, HIGH);

  pinMode (lora_NSS, OUTPUT);             //set the slave select pin as an output:
  digitalWrite(lora_NSS, HIGH);

  SPI.begin();                            //initialize SPI
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));


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
  
#ifdef ConfigureDefaults
  readSettingsDefaults();
  writeSettingsMemory();
#endif


#ifdef ConfigureFromMemory
  readSettingsMemory();
#endif

  Print_CRC_Config_Memory();

  ramc_ThisNode = ThisNode;

  printNodes();

  lora_Setup();

  if (!lora_CheckDevice())
  {
    led_Flash(100, 50);                                                //long medium speed flash for Lora device error
    Serial.println(F("LoRa Error!"));
  }

  display_current_frequency();

  ramc_CalibrationOffset = Memory_ReadInt(addr_CalibrationOffset);      //get calibration offset for this tracker
  Serial.print(F("Cal Offset "));
  Serial.println(ramc_CalibrationOffset);

  lora_Print();

  Serial.println();
  print_SupplyVoltage();
  print_Temperature();
  Serial.println();

  j = read_SupplyVoltage();                            //get supply mV
  Write_Int(0, j, lora_TXBUFF);                        //write to first two bytes of buffer
  Write_Byte(2, ramc_Current_TXconfig1, lora_TXBUFF);  //add the current config byte

  Setup_LoRaTrackerMode();
  send_Command(PowerUp);                                //send power up command, includes supply mV and config, on tracker settings
  sleepSecs(1);

#ifdef SendBind
  if (readConfigByte(TXEnable))   //is TX enabled ?
  {
    Setup_LoRaBindMode();
    sendTrackerBind();
  }
#endif

  Setup_LoRaTrackerMode();                             //so that check tone is at correct frequency

  GPS_Config_Error = false;                            //make sure GPS error flag is cleared

#ifndef DEBUGNoGPS
  GPS_On(DoGPSPowerSwitch);                            //GPS should have been on for a while by now, so this is just to start soft serial
  GPS_Setup();                                         //GPS should have had plenty of time to initialise by now

  if (GPS_Config_Error)
  {
    Serial.println(F("GPS Error !"));
    Serial.println();
    send_Command(NoGPS);                                //make sure receiver knows about GPS error
    led_Flash(100, 25);                                 //long very rapid flash for GPS error
  }
  else
  {
#ifdef CheckTone
    if (readConfigByte(TXEnable))                        //is TX enabled - needed because of fence limits
    {
      Serial.println(F("Check Tone"));                   //check tone indicates navigation model 6 set (if checktone enabled!)
      lora_Tone2(1000, 3000, 5);                         //Transmit an FM tone, 1000hz, 3000ms, 5dBm
    }
#endif
  }

  digitalWrite(LED1, HIGH);
  setStatusByte(NoGPSTestMode, 0);
  while (!gpsWaitFix(5, DontSwitch, LeaveOn))              //wait for the initial GPS fix, this could take a while, leave GPS powered on
  {

  led_Flash(2, 50);                                        //two short LED flashes to indicate GPS waiting for fix


  }
#endif

#ifndef DEBUGNoGPS
  GPS_On(DoGPSPowerSwitch);                                                 
  GPS_SetCyclicMode();                                     //set this regardless of whether hot fix mode is enabled
#endif

  lora_Tone2(500, 500, 2);                                  //Transmit an FM tone, 500hz, 500ms, 2dBm
  digitalWrite(LED1, LOW);
  sleepSecs(2);                                            //wait for GPS to shut down

#ifdef DEBUGNoGPS
  setStatusByte(NoGPSTestMode, 1);
#endif

}






