//**************************************************************************************************
// Note:
//
// Make changes to this Program file at your peril
//
// Configuration changes should be made in the Basic_Receiver_Terminal_Settingsfile not here !
// 
// This program includes the automatic frequency control functions, you can set the tracker mode
// packet to a bandwidth as low as 7.8kHz. To lock the receiver to the transmitter swap across to
// search mode to receive at least one packet, you can then go back to tracker mode. 
//
//**************************************************************************************************

#define programname "Basic_Receiver_Terminal_Beta_240118"
#define programversion "V1.2"
#define aurthorname "Stuart Robinson"
#include <Arduino.h>
#include <avr/pgmspace.h>

#include "Program_Definitions.h"
#include "Basic_Receiver_Terminal_Settings.h"

/*
**************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  To do:
  Check function of enter_CalibrationOffset()

  Changes:
  05/01/18 - Added afc functionality

  ******************************************************************************************************
*/


//Program Variables
int ramc_CalibrationOffset;                   //adjustment for frequency in Hertz

unsigned long ramc_TrackerMode_Frequency;     //Initial default for FlightFrequency
byte ramc_TrackerMode_Bandwidth;              //used to keep Tracker mode settings
byte ramc_TrackerMode_SpreadingFactor;
byte ramc_TrackerMode_CodeRate;
byte ramc_TrackerMode_Power;

unsigned long ramc_SearchMode_Frequency;      //Initial default for FlightFrequency
byte ramc_SearchMode_Bandwidth;               //used to keep Searc mode settings
byte ramc_SearchMode_SpreadingFactor;
byte ramc_SearchMode_CodeRate;
byte ramc_SearchMode_Power;

unsigned long ramc_CommandMode_Frequency;     //Initial default for command and reporting frequency
byte ramc_CommandMode_Bandwidth;              //used to keep Command mode settings
byte ramc_CommandMode_SpreadingFactor;
byte ramc_CommandMode_CodeRate;
byte ramc_CommandMode_Power;


float TRLat;                                  //tracker transmitter co-ordinates
float TRLon;
unsigned int TRAlt;
unsigned int TResets;                         //keeps the count of remote tracker resets
byte TRStatus;                                //Status byte from tracker binary payload
byte TRSats;                                  //last received number of GPS satellites from tracker
boolean Remote_GPS_Fix = false;               //set if we have received a location from the remote tracker

char keypress;                                //records the keypress from serial terminal
byte modenumber;                              //records which set of LoRa settings in use, 1 = tracker, 2 = search, 3 = command, 4=bind
byte Function_Number = 0;

boolean SD_Found = false;                     //set if SD card found at program startup

unsigned long TrackerMode_Packets;            //running count of trackermode packets received
unsigned int SearchMode_Packets;              //running count of searchmode packets received
unsigned int TestMode_Packets;                //running count of searchmode packets received
unsigned long NoFix_Packets;                  //running count of No GPS fix packets received

unsigned int SupplyVolts;

char Flight_ID[15];

//File Includes
#include Board_Definition                     //include previously defined board file
#include Memory_Library                       //include previously defined Memory Library

#include <SPI.h>
#include <Wire.h>
#include "LoRa4.h"

#ifdef UseSD
#include <SdFat.h>                            //https://github.com/greiman/SdFat
SdFat SD;
File logFile;
#endif


#ifdef USE_AFSK_RTTY_Upload
#include "AFSK_RTTY.h"
#endif

#ifdef USE_SendOnlySoftwareSerial
#include <SendOnlySoftwareSerial.h>            //https://github.com/disq/i2c-gps-nav
SendOnlySoftwareSerial Bluetooth_Serial (Bluetooth_TX);
#endif

#ifdef USE_HardwareSerial
#define Bluetooth_Serial Hardware_Bluetooth_Port
#endif

#ifdef Use_NMEA_Bluetooth_Uplink
#include "Generate_NMEA2.h"
#endif

#include "Binary2.h"
#include "PinChangeInterrupt.h"

#define max_functions 5                       //number of functions in main program loop


//**************************************************************************************************
// Eventally - the program itself starts ............
//**************************************************************************************************


void loop()
{
  Setup_LoRaTrackerMode();
  listen_LoRa(TrackerMode, 0);
  while (Serial.read() != -1);                //clear serial buffer

#ifndef NoMenu
  do
  {
    doMenu();
  } while (1);
#endif

}

void doMenu()
{
  //prints the terminal menu, caqn be ignored in handheld display mode (no serial characters to input

menustart:
  digitalWrite(LED1, LOW);                    //make sure LED is off
  Setup_LoRaTrackerMode();

  Serial.println();
  Serial.println();
  print_last_HABpacket();
  print_TRData();
  Serial.println();
  print_CurrentLoRaSettings();
  print_Nodes();
  Serial.println();
  Serial.println(F("1) Enable FSK RTTY"));
  Serial.println(F("2) Disable FSK RTTY"));
  Serial.println(F("3) Enable Address Strip"));
  Serial.println(F("4) Disable Address Strip"));
  Serial.println(F("5) Enable GPS Power Off"));
  Serial.println(F("6) Disable GPS Power Off"));
  Serial.println(F("7) Enable Fence Check"));
  Serial.println(F("8) Disable Fence Check"));
  Serial.println(F("9) Enable Doze Mode"));
  Serial.println(F("0) Disable Doze Mode"));
  Serial.println();
  Serial.println(F("B) Bind Receive"));
  Serial.println(F("C) Command Mode Listen"));
  Serial.println(F("D) Default Configuration"));
  Serial.println(F("L) Link Report"));
  Serial.println(F("O) Offset for Tracker Frequency"));
  Serial.println(F("P) Packet Test"));
  Serial.println(F("R) RTTY Test"));
  Serial.println(F("S) Search Mode Listen"));
  Serial.println(F("T) Tracker Mode Listen"));

  Serial.println(F("X) Reset Tracker Transmitter"));
  Serial.println(F("Y) Clear All Memory and Settings (re-config needed)"));
  Serial.println();
  Serial.println(F("+) Increase Transmitter Frequency 1KHZ"));
  Serial.println(F("-) Reduce Transmitter Frequency 1KHZ"));
  Serial.println();
  Serial.print(F("> "));
  delay(switch_delay);

  while (Serial.read() != -1);                //clear serial buffer

  while ((Serial.available() == 0))
  {
    //we are in menu mode so wait for a keypress or a switch press
  }

  keypress = Serial.read();                   //get the keypress

  Serial.println(keypress);
  Serial.println();

  if (keypress == '1')
  {
    Serial.println(F("Queue Enable FSKRTTY"));
    send_ConfigCommand(Config0, ThisNode, FSKRTTYEnable, 1);
  }

  if (keypress == '2')
  {
    Serial.println(F("Queue Disable FSKRTTY"));
    send_ConfigCommand(Config0, ThisNode, FSKRTTYEnable, 0);
  }

  if (keypress == '3')
  {
    Serial.println(F("Queue Enable Address Strip"));
    send_ConfigCommand(Config0, ThisNode, AddressStrip, 1);
  }

  if (keypress == '4')
  {
    Serial.println(F("Queue Disable Address Strip"));
    send_ConfigCommand(Config0, ThisNode, AddressStrip, 0);
  }

  if (keypress == '5')
  {
    //this is lat,long and alt at low data rate
    Serial.println(F("Queue Enable GPS Power Off"));
    send_ConfigCommand(Config0, ThisNode, GPSHotFix, 1);
  }

  if (keypress == '6')
  {
    //this is lat,long and alt at low data rate
    Serial.println(F("Queue Disable GPS Power Off"));
    send_ConfigCommand(Config0, ThisNode, GPSHotFix, 0);
  }

  if (keypress == '7')
  {
    Serial.println(F("Queue Enable Fence Check"));
    send_ConfigCommand(Config0, ThisNode, CheckFence, 1);
  }


  if (keypress == '8')
  {
    Serial.println(F("Queue Disable Fence Check"));
    send_ConfigCommand(Config0, ThisNode, CheckFence, 0);
  }

  if (keypress == '9')
  {
    Serial.println(F("Queue Enable Doze"));
    send_ConfigCommand(Config0, ThisNode, DozeEnable, 1);
  }

  if (keypress == '0')
  {
    Serial.println(F("Queue Disable Doze"));
    send_ConfigCommand(Config0, ThisNode, DozeEnable, 0);
  }


  if ((keypress == 'B') || (keypress == 'b'))
  {
    Function_Number = 5;                                       //bind only accepted in function 5
    listen_LoRa(BindMode, 0);
    Serial.println(F("Exit Bind Mode"));
  }

  if ((keypress == 'C') || (keypress == 'c'))
  {
    Function_Number = 0;
    listen_LoRa(CommandMode, 0);
  }


  if ((keypress == 'L') || (keypress == 'l'))
  {
    lora_TXBUFF[0] = '0';
    Serial.println(F("Queue Link Report"));
    lora_QueuedSend(0, 0, LinkReport, Broadcast, ThisNode, 10, lora_Power, default_attempts, NoStrip);  //send a Link Report Request
    delay(inter_Packet_delay);
    listen_LoRa(CommandMode, 0);                              //listen for the link budget reply
  }

  if ((keypress == 'O') || (keypress == 'o'))
  {
    enter_CalibrationOffset();
  }

  if ((keypress == 'P') || (keypress == 'p'))
  {
    Serial.println(F("Queue LoRa Packet Test"));
    send_TestRequest('0');
    listen_LoRa(TrackerMode, 0);                              //command has been sent so listen for replies
  }

  if ((keypress == 'R') || (keypress == 'r'))
  {
    Serial.println(F("Queue FSK RTTY Test Request"));
    send_TestRequest('1');
  }

  if ((keypress == 'S') || (keypress == 's'))
  {
    Function_Number = 2;
    listen_LoRa(SearchMode, 0);
  }

  if ((keypress == 'T') || (keypress == 't'))
  {
    Function_Number = 1;
    listen_LoRa(TrackerMode, 0);
  }


  if (keypress == 'D')
  {
    read_Settings_Defaults();
    write_Settings_Memory();
    Serial.println();
  }

  if (keypress == 'Y')
  {
    Clear_All_Memory();
    read_Settings_Memory();
  }

  if (keypress == 'X')
  {
    load_key();
    Serial.println(F("Queue Reset Tracker"));
    lora_QueuedSend(0, 3, ResetTracker, Broadcast, ThisNode, 10, lora_Power, default_attempts, NoStrip);  //send a reset request
  }

  if (keypress == '+')
  {
    Serial.println(F("Increase Tracker Frequency 1KHZ"));
    lora_QueuedSend(0, 1, INCFreq, Broadcast, ThisNode, 10, lora_Power, default_attempts, NoStrip);
  }

  if (keypress == '-')
  {
    Serial.println(F("Decrease Tracker Frequency 1KHZ"));
    lora_QueuedSend(0, 1, DECFreq, Broadcast, ThisNode, 10, lora_Power, default_attempts, NoStrip);
  }

  goto menustart;                                  //all programs deserve at least one goto, if only because it works
}


void check_AFC()
{
  signed int freqerror;
  freqerror = lora_GetFrequencyError();
  Serial.print(F("  FreqErr,"));
  Serial.print(freqerror);
  ramc_CalibrationOffset = ramc_CalibrationOffset - freqerror;
  Serial.print(F("  CalOffset,"));
  Serial.print(ramc_CalibrationOffset);
  lora_SetFreq(lora_CurrentFreq, ramc_CalibrationOffset);
}



void enter_CalibrationOffset()
{
  //allows calibration offset of recever to be changed from terminal keyboard
  float tempfloat;

  Serial.println(F("Enter Khz offset > "));
  while (Serial.available() == 0);
  {
    tempfloat = Serial.parseFloat();
    Serial.print(F("Offset = "));
    Serial.println(tempfloat, 5);
  }

  while (Serial.read() != -1);                //clear serial buffer

}


void load_key()
{
  //loads the protection key to first locations in TX buffer
  lora_TXBUFF[0] = key0;
  lora_TXBUFF[1] = key1;
  lora_TXBUFF[2] = key2;
  lora_TXBUFF[3] = key3;
}


boolean Is_Key_Valid()
{
  //checks if protection key in RX bauffer matches

  Serial.print(F("Received Key "));
  Serial.write(lora_RXBUFF[0]);
  Serial.write(lora_RXBUFF[1]);
  Serial.write(lora_RXBUFF[2]);
  Serial.write(lora_RXBUFF[3]);
  Serial.println();

  if ( (lora_RXBUFF[0] == key0) && (lora_RXBUFF[1] == key1)  && (lora_RXBUFF[2] == key2)  && (lora_RXBUFF[3] == key3) )
  {
    Serial.println(F("Key Valid"));
    return true;
  }
  else
  {
    Serial.println(F("Key Not Valid"));
    return false;
  }
}


byte listen_LoRa(byte lmode, unsigned long listen_mS)
{
  //listen for packets in specific mode, timeout in mS, 0 = continuous wait
  unsigned long listen_endmS;

  switch (lmode)
  {
    case TrackerMode:
      Setup_LoRaTrackerMode();
      break;

    case SearchMode:
      Setup_LoRaSearchMode();
      break;

    case CommandMode:
      Setup_LoRaCommandMode();
      break;

    case BindMode:
      Setup_LoRaBindMode();
      break;
  }

  print_CurrentLoRaSettings();
  listen_endmS = (millis() + listen_mS);                    //calculate the millis to stop listing at.

  keypress = 0;

  while (Serial.read() != -1);                              //empty serial buffer

  Serial.println();
  print_mode(modenumber);
  Serial.print(F("Listen > "));
  lora_RXpacketCount = 0;
  keypress = 0;
  lora_RXONLoRa();

  do
  {
    check_for_Packet();

    if (listen_mS > 0)
    {
      if (millis() >= listen_endmS)
      {
        Serial.println(F("Listen Timeout"));
        break;
      }
    }
  }
  while ((Serial.available() == 0));

  if (Serial.available() != 0)
  {
    keypress = Serial.read();
  }

  if (keypress > 0)
  {
    Serial.println(F("Key Press"));
  }

  if ( (millis() > listen_endmS)  && (listen_mS != 0) )
  {
    Serial.println(F("Timeout"));
    keypress = 1;
  }

  return keypress;
}


void print_mode(byte lmode)
{
  switch (lmode)
  {
    case TrackerMode:
      Serial.print(F("Tracker Mode "));
      break;

    case SearchMode:
      Serial.print(F("Search Mode "));
      break;

    case CommandMode:
      Serial.print(F("Command Mode "));
      break;

    case BindMode:
      Serial.print(F("Bind Mode "));
      break;
  }
}


void send_TestRequest(char cmd)
{
  //transmit a test request
  Serial.print(F("Send Test Request "));
  Serial.write(cmd);
  Serial.println();

  lora_TXBUFF[0] = cmd;
  lora_QueuedSend(0, 0, Test, Broadcast, ThisNode, 10, lora_Power, default_attempts, NoStrip);   //send a Test Request
  delay(inter_Packet_delay);
}


void send_ConfigCommand(char confignum, char configdestination, char bitnum, char bitval)
{
  //transmits a config command
  byte returnbyte;
  Setup_LoRaCommandMode();
  bitnum = bitnum + 48;
  bitval = bitval + 48;
  lora_TXBUFF[0] = bitnum;            //set the bitnum
  lora_TXBUFF[1] = bitval;            //set the bitval

  returnbyte = lora_QueuedSend(0, 1, Config0, configdestination, ThisNode, 10, lora_Power, default_attempts, NoStrip);

  if (returnbyte == 1)
  {
    Serial.println(F("Config Sent OK"));
    print_TrackerLastSNR();
    print_TrackerLastRSSI();
  }
  else
  {
    Serial.println(F("Config Send Failed"));
  }
  Serial.println();
  delay(inter_Packet_delay);
}


void print_TrackerLastSNR()
{
  int8_t temp;
  temp =   Read_Byte(0, lora_RXBUFF);
  Serial.print(F("Last Tracker SNR "));
  Serial.print(temp);
  Serial.println(F("dB"));
}


void print_TrackerLastRSSI()
{
  int8_t temp;
  temp = Read_Byte(1, lora_RXBUFF);
  Serial.print(F("Last Tracker RSSI "));
  Serial.print(temp);
  Serial.println(F("dB"));
}


byte check_for_Packet()
{
  //checks to see if a packet has arrived
  byte returnbyte;

#ifdef No_DIO0_RX_Ready
  returnbyte = lora_readRXready();
#else
  returnbyte = lora_readRXready2();
#endif

  if (returnbyte == 64)
  {
    digitalWrite(LED1, HIGH);
    lora_ReadPacket();
    lora_AddressInfo();
    lora_ReceptionInfo();

#ifdef LORA_AFC
    check_AFC();
#endif

    Serial.println();
    process_Packet();
    digitalWrite(LED1, LOW);
    Serial.println();
    print_mode(modenumber);
    Serial.print(F("Listen > "));
    lora_RXONLoRa();                                //ready for next and clear flags
    return 1;
  }

  if (returnbyte == 96)
  {
    Serial.println(F("CRC Error"));
    Serial.println();
    print_mode(modenumber);
    Serial.print(F("Listen > "));
    lora_RXONLoRa();                                //ready for next
  }

  return 0;
}


void write_HABPacketMemory(byte RXStart, byte RXEnd)
{
  //stores the HAB packet in memory, FRAM or EEPROM
  byte index, bufferdata;
  unsigned int MemAddr = addr_StartHABPayloadData;                 //address in FRAM where last received HAB payload stored

  Memory_WriteByte(MemAddr, lora_RXPacketType);
  MemAddr++;
  Memory_WriteByte(MemAddr, lora_RXDestination);
  MemAddr++;
  Memory_WriteByte(MemAddr, lora_RXSource);
  MemAddr++;
  for (index = RXStart; index <= RXEnd; index++)
  {
    bufferdata = lora_RXBUFF[index];
    Memory_WriteByte(MemAddr, bufferdata);
    MemAddr++;
  }
  Memory_WriteByte(MemAddr, 0xFF);
}


void print_FightID()
{
  //send flight ID to serila terminal
  byte index, bufferdata;
  index = 0;
  do
  {
    bufferdata = Flight_ID[index++];
    Serial.write(bufferdata);
  }
  while (bufferdata != 0);
}


void extract_HABPacket(byte passedRXStart, byte passedRXEnd)
{
  //extracts data from received HAB packets where first fields are lat,lon,alt.
  byte tempbyte;
  byte savedRXStart;

  savedRXStart = lora_RXStart;                 //save current value of lora_RXStart
  lora_RXStart = passedRXStart;                //use lora_RXStart as its global

  //Skip leading $
  do
  {
    tempbyte =  lora_RXBUFF[lora_RXStart++];
  }
  while ( tempbyte == '$');
  lora_RXStart--;

  //ID
  tempbyte = sizeof(Flight_ID);

  extract_Buffer(Flight_ID, sizeof(Flight_ID));

  print_FightID();
  Serial.print(F(" "));

  lora_RXStart = next_Comma(lora_RXStart);
  lora_RXStart = next_Comma(lora_RXStart);

  //Lat
  TRLat = extract_Float();

  //Lon
  TRLon = extract_Float();

  //Alt
  TRAlt = extract_Uint();

  print_Tracker_Location();
  Serial.println();

  TRSats = extract_Uint();
  lora_RXStart = next_Comma(lora_RXStart);
  lora_RXStart = next_Comma(lora_RXStart);

  //Reset
  TResets = extract_Uint();
  lora_RXStart = next_Comma(lora_RXStart);
  TRStatus = extract_Uint();
  lora_RXStart = savedRXStart;                    //restore lora_RXStart, just in case
}


float extract_Float()
{
  //extracts a float in ASCII format from buffer
  char temp[12];
  byte tempptr = 0;
  byte bufferdata;
  float tempfloat;
  do
  {
    bufferdata =  lora_RXBUFF[lora_RXStart++];
    temp[tempptr++] = bufferdata;
  }
  while (bufferdata != ',');
  temp[tempptr] = 0;  //terminator for string
  tempfloat = (float)atof(temp);
  return tempfloat;
}


void extract_Buffer(char * mybuffer, size_t bufSize)
{
  //extracts a buffer in ASCII format from RXbuffer
  byte index, buffersize;

  buffersize = bufSize - 1;
  for (index = 0; index <= buffersize; index++)
  {
    mybuffer[index] = lora_RXBUFF[lora_RXStart++];
    if (mybuffer[index] == ',')
    {
      break;
    }
  }
  mybuffer[index] = 0;
}


float extract_Uint()
{
  //extracts an unsigned int in ASCII format from buffer
  char temp[12];
  byte tempptr = 0;
  byte buffdata;
  unsigned int tempint;
  do
  {
    buffdata =  lora_RXBUFF[lora_RXStart++];
    temp[tempptr++] = buffdata;
  }
  while (buffdata != ',');
  temp[tempptr] = 0;  //terminator for string
  tempint = (unsigned int)atof(temp);
  return tempint;
}


byte next_Comma(byte localpointer)
{
  //skips through HAB packet (in CSV format) to next  comma
  byte bufferdata;
  do
  {
    bufferdata =  lora_RXBUFF[localpointer++];
  }
  while (bufferdata != ',');
  return localpointer;
}


/**********************************************************************
  Tracker data routines
***********************************************************************
*/

void extract_TRbinarylocationData()
{
  //extracts the binary location data from receive buffer and records date and time

  TRLat = Read_Float(0, lora_RXBUFF);
  TRLon = Read_Float(4, lora_RXBUFF);
  TRAlt = Read_UInt(8, lora_RXBUFF);
}


void print_TRData()
{
  //prints the last received tracker location data
  Serial.print(F("Last Location  "));
  Serial.print(TRLat, 5);
  Serial.print(F("  "));
  Serial.print(TRLon, 5);
  Serial.print(F("  "));
  Serial.println(TRAlt);
}


void save_TRData()
{
  //writes the last received tracker location data to memory
  Memory_WriteFloat(addr_TRLat, TRLat);
  Memory_WriteFloat(addr_TRLon, TRLon);
  Memory_WriteUInt(addr_TRAlt, TRAlt);

}


void read_TRData()
{
  //read stored tracker location data from memory
  TRLat = Memory_ReadFloat(addr_TRLat);
  TRLon = Memory_ReadFloat(addr_TRLon);
  TRAlt = Memory_ReadUInt(addr_TRAlt);
}


void clear_TRData()
{
  //sets the tracker location data to zero
  TRLat = 0;
  TRLon = 0;
  TRAlt = 0;
}


void print_Tracker_Location()
{
  //prints the tracker location data only
  Serial.print(TRLat, 6);
  Serial.print(F(","));
  Serial.print(TRLon, 6);
  Serial.print(F(","));
  Serial.print(TRAlt);
}


void print_packet_HEX(byte RXStart, byte RXEnd)
{
  //prints contents of received packet as hexadecimal
  byte bufferdata;
  Serial.print(lora_RXPacketType, HEX);
  Serial.print(F(" "));
  Serial.print(lora_RXDestination, HEX);
  Serial.print(F(" "));
  Serial.print(lora_RXSource, HEX);
  Serial.print(F(" "));

  for (byte index = RXStart; index <= RXEnd; index++)
  {
    bufferdata = lora_RXBUFF[index];
    if (bufferdata < 0x10)
    {
      Serial.print(F("0"));
    }
    Serial.print(bufferdata, HEX);
    Serial.print(F(" "));
  }

}


void process_Packet()
{
  //process and decide what to do with received packet
  unsigned int index, tempint;
  byte tempbyte, ptr;
  unsigned int returnedCRC;
  int8_t tempchar;

  digitalWrite(LED1, HIGH);

  if (lora_RXPacketType == LocationBinaryPacket)
  {
    Remote_GPS_Fix = true;

    if (modenumber == SearchMode)
    {
      SearchMode_Packets++;
    }

    if (modenumber == TrackerMode)
    {
      TrackerMode_Packets++;
    }

    extract_TRbinarylocationData();
    save_TRData();

    print_Tracker_Location();
    TRStatus = Read_Byte(10, lora_RXBUFF);
    Serial.print(F(","));
    print_AllBits(TRStatus);
    Serial.println();


#ifdef UseSD
    SD_WriteBinarypacket_Log();
#endif


    digitalWrite(LED1, LOW);

#ifdef Use_NMEA_Bluetooth_Uplink
    Serial.println();
    send_NMEA(TRLat, TRLon, TRAlt);                    //Send position to Bluetooth, sends two NMEA strings
    Serial.println();
#endif

    display_fix_Status();
    return;
  }


  if (lora_RXPacketType == HABPacket)
  {
    Remote_GPS_Fix = true;
    TrackerMode_Packets++;

    lora_RXBuffPrint(PrintASCII);                      //print packet contents as ASCII
    Serial.println();

#ifdef UseSD
    SD_WriteHABpacket_Log();
#endif

    write_HABPacketMemory(lora_RXStart, lora_RXEnd);
    extract_HABPacket(lora_RXStart, lora_RXEnd);

    save_TRData();

    digitalWrite(LED1, LOW);

#ifdef Use_NMEA_Bluetooth_Uplink
    send_NMEA(TRLat, TRLon, TRAlt);                      //Send position to Bluetooth
    Serial.println();
#endif

#ifdef USE_AFSK_RTTY_Upload
    Serial.print(F("AFSK RTTY Upload "));
    Serial.println();
    start_AFSK_RTTY();

    for (index = 0; index <= 3; index++)
    {
      SendAFSKRTTY('$');
    }

    for (index = lora_RXStart; index <= lora_RXEnd; index++)
    {
      SendAFSKRTTY(lora_RXBUFF[index]);
    }
    SendAFSKRTTY(13);
    SendAFSKRTTY(10);
    end_AFSK_RTTY();
#endif

    display_fix_Status();
    return;
  }


  if (lora_RXPacketType == Testpacket)
  {
    Serial.print(F("TestPacket "));
    TestMode_Packets++;
    lora_RXBuffPrint(PrintASCII);                        //print packet contents as ASCII
    Serial.println();
    return;
  }


#ifndef NoMenu
  if (lora_RXPacketType == Bind )
  {

    Serial.println(F("Tracker Bind Received"));
    ptr = 4;                         //set pointer to start of Bind data in lora_RXBUFF

    if (!Is_Key_Valid())
    {
      return;
    }

    if ((Function_Number != 5))      //only accept incoming bind request when in function 5
    {
      Serial.println(F("Not in Bind Mode"));
      return;
    }

    Print_CRC_Bind_Memory();

    tempint = (lora_RXBUFF[lora_RXEnd] * 256) + (lora_RXBUFF[lora_RXEnd - 1]);

    Serial.print(F("Transmitted CRC "));
    Serial.println(tempint, HEX);
    returnedCRC = RXBuffer_CRC(ptr, (lora_RXEnd - 2));
    Serial.print(F("Received CRC "));
    Serial.println(returnedCRC, HEX);


    if (returnedCRC == tempint)
    {
      Serial.println(F("Accepted"));

      for (index = addr_StartBindData; index <= addr_EndBindData; index++)
      {
        tempbyte = lora_RXBUFF[ptr++];
        Memory_WriteByte(index, tempbyte);
      }
      read_Settings_Memory();             //now bring the new settings into use
      Print_CRC_Bind_Memory();
    }
    else
    {
      Serial.println(F("Rejected"));
      return;
    }
  }
#endif


  if (lora_RXPacketType == Wakeup)
  {
    Serial.write(7);                                       //print a bell
    Serial.println();
    return;
  }


  if (lora_RXPacketType == NoFix)
  {
    NoFix_Packets++;
    Serial.print(F("No Tracker GPS Fix "));
    Serial.println(NoFix_Packets);
    return;
  }

  if (lora_RXPacketType == NoGPS)
  {
    Serial.print(F("Tracker GPS Error "));
    return;
  }


  if (lora_RXPacketType == PowerUp)
  {
    tempint = Read_UInt(2, lora_RXBUFF);
    Serial.print(F("Power Up "));
    Serial.print(tempint);
    Serial.println(F("mV"));
    return;
  }


  if (lora_RXPacketType == Info)
  {
    Serial.println();
    Serial.print(F("Tracker Last Packet Reception  SNR,"));
    tempchar = Read_Byte(0, lora_RXBUFF);
    Serial.print(tempchar);
    Serial.print(F("dB"));
    tempchar = Read_Byte(1, lora_RXBUFF);
    Serial.print(F("  RSSI,"));
    Serial.print(tempchar);
    Serial.println(F("dB"));
    Serial.print(F("Battery,"));
    tempint = Read_Int(2, lora_RXBUFF);
    Serial.print(tempint);
    Serial.print(F("mV,TRStatus,"));
    tempbyte = (byte) Read_Int(4, lora_RXBUFF);
    print_AllBits(tempbyte);
    Serial.println();
    Serial.println();
    return;
  }


  if (lora_RXPacketType == ClearToSendCommand)
  {
    return;                                 //do nothing
  }


  if (lora_RXPacketType == ClearToSend)
  {
    return;                                 //do nothing
  }


  if (lora_RXPacketType == Sensor1)
  {
    Serial.println(F("Sensor1 Packet"));
    process_Sensor1();
  }
}


unsigned int RXBuffer_CRC(unsigned int startaddr, unsigned int endaddr)
{
  unsigned int i, CRC;

  CRC = 0xffff;                                              //start value for CRC16
  byte j;

  for (i = startaddr; i <= endaddr; i++)                     //element 4 is first character after $$$$ at start
  {
    CRC ^= ((uint16_t) lora_RXBUFF[i] << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }
  return CRC;

}


void process_Sensor1()
{
  float lTemperature, lHumidity, lPressure, lAltitude;

  lTemperature = Read_Float(0, lora_RXBUFF);
  lHumidity = Read_Float(4, lora_RXBUFF);
  lPressure = Read_Float(8, lora_RXBUFF);
  lAltitude = Read_Float(12, lora_RXBUFF);

  Serial.print(F("Temperature: "));
  Serial.print(lTemperature, 2);
  Serial.println(F(" degrees C"));

  Serial.print(F("%RH: "));
  Serial.print(lHumidity, 2);
  Serial.println(F("  %"));

  Serial.print(F("Pressure: "));
  Serial.print(lPressure, 2);
  Serial.println(F(" Pa"));

  Serial.print(F("Altitude: "));
  Serial.print(lAltitude, 2);
  Serial.println(F("m"));
}


void print_AllBits(byte myByte)
{
  //prints bits of byte as binary
  for (byte mask = 0x80; mask; mask >>= 1) {
    if (mask  & myByte)
      Serial.print('1');
    else
      Serial.print('0');
  }
}


void display_fix_Status()
{
  //display fix status of tracker

  if (!(bitRead(TRStatus, GPSFix)))
  {
    Serial.print(F("No "));
  }

  Serial.println(F("Tracker GPS fix"));
}


#ifdef UseSD
void SD_WriteHABpacket_Log()
{
  //write HAB packet to SD card
  byte index;

  if (SD_Found)
  {
    logFile.write(lora_RXPacketType);
    logFile.write(lora_RXDestination);
    logFile.write(lora_RXSource);
    logFile.print(F(","));

    for (index = lora_RXStart; index <= lora_RXEnd; index++)
    {
      logFile.write(lora_RXBUFF[index]);
    }
    logFile.write(13);
    logFile.write(10);
    logFile.flush();
  }
  else
  {
    Serial.println("No SD card !");
  }
}


void SD_WriteBinarypacket_Log()
{
  //send the location data in binary packet to the log
  if (SD_Found)
  {
    logFile.write(lora_RXPacketType);
    logFile.write(lora_RXDestination);
    logFile.write(lora_RXSource);
    logFile.print(F(","));
    logFile.print(F(","));
    SD_addLatLonAlt_Log();

    logFile.write(13);
    logFile.write(10);
    logFile.flush();
  }
  else
  {
    Serial.println(F("No SD card !"));
  }
}


void SD_addLatLonAlt_Log()
{
  logFile.print(TRLat, 6);
  logFile.print(F(","));
  logFile.print(TRLon, 6);
  logFile.print(F(","));
  logFile.print(TRAlt, 6);
  logFile.write(13);
  logFile.write(10);
}


boolean setup_SDLOG()
{
  //checks if the SD card is present and can be initialised

  Serial.print(F("SD card..."));

  if (!SD.begin(SD_CS))
  {
    Serial.println(F("Failed, or not present"));
    SD_Found = false;
    return false;                         //don't do anything more:
  }

  Serial.print(F("Initialized OK"));
  SD_Found = true;

  char filename[] = "Track000.txt";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logFile = SD.open(filename, FILE_WRITE);
      break;                            // leave the loop!
    }
  }

  Serial.print(F("...Writing to "));
  Serial.print(F("Track0"));
  Serial.write(filename[6]);
  Serial.write(filename[7]);
  Serial.print(F(".txt"));
  return true;
}


void printlog_addleadingZero(byte temp)
{
  if (temp < 10)
  {
    logFile.print(F("0"));
  }
  logFile.print(temp);
}
#endif


void print_CurrentLoRaSettings()
{
  //prints the current LoRa settings, reads device registers
  float tempfloat;
  //int tempint;
  byte regdata;
  unsigned long bw;

  tempfloat = lora_GetFreq();
  Serial.print(tempfloat, 3);
  Serial.print(F("MHz  ("));

  Serial.print(ramc_CalibrationOffset);
  Serial.print(F("hz)"));

  regdata = lora_Read(lora_RegModemConfig1);
  regdata = regdata & 0xF0;
  bw = lora_returnbandwidth(regdata);
  Serial.print(F("  BW"));
  Serial.print(bw);

  regdata = lora_Read(lora_RegModemConfig2);
  regdata = (regdata & 0xF0) / 16;
  Serial.print(F("  SF"));
  Serial.print(regdata);

  regdata = lora_Read(lora_RegModemConfig1);
  regdata = regdata & B00001110;
  regdata = regdata / 2; //move right one
  regdata = regdata + 4;

  Serial.print(F("  CR4/"));
  Serial.print(regdata);

  regdata = lora_Read(lora_RegModemConfig3);
  regdata = regdata & B00001000;
  Serial.print(F("  LDROPT_"));
  if (regdata == 8)
  {
    Serial.print(F("ON"));
  }
  else
  {
    Serial.print(F("OFF"));
  }

  regdata = lora_Read(lora_RegPaConfig);
  regdata = regdata - 0xEE;

  Serial.print(F("  Power "));
  Serial.print(regdata);
  Serial.print(F("dBm"));
  Serial.println();
}


void system_Error()
{
  //there is a error, likley no LoRa device found, cannot continue
  while (1)
  {
    digitalWrite(LED1, HIGH);
    delay(50);
    digitalWrite(LED1, LOW);
    delay(50);
  }
}


void led_Flash(unsigned int flashes, unsigned int dealymS)
{
  //flash LED to show tracker is alive
  unsigned int index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(dealymS);
    digitalWrite(LED1, LOW);
    delay(dealymS);
  }
}


void read_Settings_Defaults()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this receiver program uses constants in RAM copied from memory in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver.
  Serial.println(F("Configuring Settings from Defaults"));
  ramc_CalibrationOffset = CalibrationOffset;
  ramc_TrackerMode_Frequency = TrackerMode_Frequency;
  ramc_CommandMode_Frequency = CommandMode_Frequency;
  ramc_SearchMode_Frequency = SearchMode_Frequency;
  ramc_TrackerMode_Bandwidth = TrackerMode_Bandwidth;
  ramc_TrackerMode_SpreadingFactor = TrackerMode_SpreadingFactor;
  ramc_TrackerMode_CodeRate = TrackerMode_CodeRate;
  ramc_CommandMode_Bandwidth = CommandMode_Bandwidth;
  ramc_CommandMode_SpreadingFactor = CommandMode_SpreadingFactor;
  ramc_CommandMode_CodeRate = CommandMode_CodeRate;
  ramc_SearchMode_Bandwidth = SearchMode_Bandwidth;
  ramc_SearchMode_SpreadingFactor = SearchMode_SpreadingFactor;
  ramc_SearchMode_CodeRate = SearchMode_CodeRate;
  ramc_TrackerMode_Power = TrackerMode_Power;
  ramc_SearchMode_Power = SearchMode_Power;
  ramc_CommandMode_Power = CommandMode_Power;
}


void read_Settings_Memory()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this receiver program uses constants in RAM copied from memory in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver.
  Serial.println(F("Configuring Settings from Memory"));
  ramc_CalibrationOffset = Memory_ReadInt(addr_CalibrationOffset);
  ramc_TrackerMode_Frequency = Memory_ReadULong(addr_TrackerMode_Frequency);
  ramc_CommandMode_Frequency = Memory_ReadULong(addr_CommandMode_Frequency);
  ramc_SearchMode_Frequency = Memory_ReadULong(addr_SearchMode_Frequency);
  ramc_TrackerMode_Bandwidth = Memory_ReadByte(addr_TrackerMode_Bandwidth);
  ramc_TrackerMode_SpreadingFactor = Memory_ReadByte(addr_TrackerMode_SpreadingFactor);
  ramc_TrackerMode_CodeRate = Memory_ReadByte(addr_TrackerMode_CodeRate);
  ramc_CommandMode_Bandwidth = Memory_ReadByte(addr_CommandMode_Bandwidth);
  ramc_CommandMode_SpreadingFactor = Memory_ReadByte(addr_CommandMode_SpreadingFactor);
  ramc_CommandMode_CodeRate = Memory_ReadByte(addr_CommandMode_CodeRate);
  ramc_SearchMode_Bandwidth = Memory_ReadByte(addr_SearchMode_Bandwidth);
  ramc_SearchMode_SpreadingFactor = Memory_ReadByte(addr_SearchMode_SpreadingFactor);
  ramc_SearchMode_CodeRate = Memory_ReadByte(addr_SearchMode_CodeRate);
  ramc_TrackerMode_Power = Memory_ReadByte(addr_TrackerMode_Power);
  ramc_SearchMode_Power = Memory_ReadByte(addr_SearchMode_Power);
  ramc_CommandMode_Power = Memory_ReadByte(addr_CommandMode_Power);
}


void write_Settings_Memory()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this receiver program uses constants in RAM copied from memory in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver..
  Serial.println(F("Writing RAM Settings to Memory"));
  Memory_Set(addr_StartConfigData, addr_EndConfigData, 0);          //clear memory area first
  Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);
  Memory_WriteULong(addr_TrackerMode_Frequency, ramc_TrackerMode_Frequency);
  Memory_WriteULong(addr_CommandMode_Frequency, ramc_CommandMode_Frequency);
  Memory_WriteULong(addr_SearchMode_Frequency, ramc_SearchMode_Frequency);
  Memory_WriteByte(addr_TrackerMode_Bandwidth, ramc_TrackerMode_Bandwidth);
  Memory_WriteByte(addr_TrackerMode_SpreadingFactor, ramc_TrackerMode_SpreadingFactor);
  Memory_WriteByte(addr_TrackerMode_CodeRate, ramc_TrackerMode_CodeRate);
  Memory_WriteByte(addr_CommandMode_Bandwidth, ramc_CommandMode_Bandwidth);
  Memory_WriteByte(addr_CommandMode_SpreadingFactor, ramc_CommandMode_SpreadingFactor);
  Memory_WriteByte(addr_CommandMode_CodeRate, ramc_CommandMode_CodeRate);
  Memory_WriteByte(addr_SearchMode_Bandwidth, ramc_SearchMode_Bandwidth);
  Memory_WriteByte(addr_SearchMode_SpreadingFactor, ramc_SearchMode_SpreadingFactor);
  Memory_WriteByte(addr_SearchMode_CodeRate, ramc_SearchMode_CodeRate);
  Memory_WriteByte(addr_TrackerMode_Power, ramc_TrackerMode_Power);
  Memory_WriteByte(addr_SearchMode_Power, ramc_SearchMode_Power);
  Memory_WriteByte(addr_CommandMode_Power, ramc_CommandMode_Power);
}


void Serialprint_addleadingZero(byte temp)
{
  if (temp < 10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp);
}

//*******************************************************************************************************
// Memory Routines
//*******************************************************************************************************


void Clear_All_Memory()
{
  //clears the whole of memory, normally 1kbyte
  Serial.print(F("Clearing All Memory"));
  Memory_Set(addr_StartMemory, addr_EndMemory, 0);
}


void Print_CRC_Bind_Memory()
{
  unsigned int returnedCRC = Memory_CRC(addr_StartBindData, addr_EndBindData);
  Serial.print(F("CRC_Bind_Memory "));
  Serial.println(returnedCRC, HEX);
}


//*******************************************************************************************************


void print_Nodes()
{
  //prints current node of this device
  Serial.print(F("This Node "));
  Serial.write(ThisNode);
  Serial.println();
}


void print_last_HABpacket()
{
  //prints last received HAB packet to serial terminal
  byte memorydata;
  unsigned int address;
  address = addr_StartHABPayloadData;

  Serial.print(F("Last HAB Packet  "));
  do
  {
    memorydata = Memory_ReadByte(address);
    if ((memorydata == 0xFF) || (address >= addr_EndHABPayloadData))
    {
      break;
    }
    Serial.write(memorydata);
    address++;
  }
  while (true);
  Serial.println();
}


void Setup_LoRaTrackerMode()
{
  //sets LoRa modem to Tracker mode
  lora_SetFreq(ramc_TrackerMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_TrackerMode_Bandwidth, ramc_TrackerMode_SpreadingFactor, ramc_TrackerMode_CodeRate, Explicit);  //Setup the LoRa modem parameters
  lora_Power = ramc_TrackerMode_Power;
  modenumber = TrackerMode;
}


void Setup_LoRaSearchMode()
{
  //sets LoRa modem to Search mode
  lora_SetFreq(ramc_SearchMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_SearchMode_Bandwidth, ramc_SearchMode_SpreadingFactor, ramc_SearchMode_CodeRate, Explicit);  //Setup the LoRa modem parameters
  lora_Power = ramc_SearchMode_Power;
  modenumber = SearchMode;
}

void Setup_LoRaCommandMode()
{
  //sets LoRa modem to Command mode
  lora_SetFreq(ramc_CommandMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_CommandMode_Bandwidth, ramc_CommandMode_SpreadingFactor, ramc_CommandMode_CodeRate, Explicit);  //Setup the LoRa modem parameters
  lora_Power = ramc_CommandMode_Power;
  modenumber = CommandMode;
}


void Setup_LoRaBindMode()
{
  //sets LoRa modem to Bind mode
  lora_SetFreq(BindMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(BindMode_Bandwidth, BindMode_SpreadingFactor, BindMode_CodeRate, Explicit); //Setup the LoRa modem parameters
  lora_Power = BindMode_Power;
  modenumber = BindMode;
}


void display_frequency()
{
  //display current set frequency of LoRa device
  float freq_temp;
  freq_temp = lora_GetFreq();
  Serial.print(F("Set to Frequency "));
  Serial.print(freq_temp, 3);
  Serial.println(F("MHz"));
}



void display_frequencies_memory()
{
  //display the frequncies set in memory, good check to see if all is well with memory
  unsigned long freq_temp;
  freq_temp = Memory_ReadULong(addr_TrackerMode_Frequency);
  Serial.print(F("TrackerMode "));
  Serial.println(freq_temp);
  freq_temp = Memory_ReadULong(addr_SearchMode_Frequency);
  Serial.print(F("SearchMode  "));
  Serial.println(freq_temp);
  freq_temp = Memory_ReadULong(addr_CommandMode_Frequency);
  Serial.print(F("CommandMode "));
  Serial.println(freq_temp);
  Serial.println();

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


void setup()
{
  //needs no explanation I hope ...............

  pinMode(LED1, OUTPUT);                    //setup pin for PCB LED
  led_Flash(2, 250);

  Serial.begin(38400);                       //setup Serial console ouput
  Serial.println();
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(aurthorname));
  Serial.println();

  SPI.begin();                               //initialize SPI:
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  pinMode(lora_NReset, OUTPUT);              //LoRa Device reset line
  pinMode (lora_NSS, OUTPUT);                //LoRa Device select line
  digitalWrite(lora_NSS, HIGH);
  digitalWrite(lora_NReset, HIGH);
  delay(1);

  Memory_Start();

#ifdef ClearAllMemory
  Clear_All_Memory();
  Serial.println();
  Serial.println();
#endif


#ifdef ClearConfigData
  Clear_Config_Memory();
#endif


#ifdef ConfigureDefaults
  read_Settings_Defaults();
  write_Settings_Memory();
  Serial.println();
  Serial.flush();
#endif

#ifdef ConfigureFromMemory
  read_Settings_Memory();
#endif

  Print_CRC_Bind_Memory();

  read_TRData();

  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));   //redo faster SPI as changed by display_setup

  if (!lora_CheckDevice())
  {
    Serial.println(F("LoRa Error!"));
    led_Flash(40, 50);
  }

  lora_Setup();

#ifdef Use_NMEA_Bluetooth_Uplink
  Bluetooth_Serial_Setup();
#endif

  print_Nodes();
  Serial.println();

  Setup_LoRaTrackerMode();

  Serial.println();
  lora_Print();
  Serial.println();
  Serial.flush();

#ifdef CalibrateTone
  digitalWrite(LED1, HIGH);                   //turn on LED
  lora_Tone(1000, 3000, 10);                  //Transmit an FM tone, 1000hz, 1000ms, 10dBm
  digitalWrite(LED1, LOW);                    //LED is off
  delay(1000);
#endif

  lora_RXONLoRa();

#ifdef UseSD
  setup_SDLOG();                               //setup SD and delay a bit to ensure any pending ints cleared
#endif

  Serial.println();
  print_last_HABpacket();
  print_TRData();
  Serial.println();

  Serial.println();
  display_frequencies_memory();

#ifdef Use_NMEA_Bluetooth_Uplink
  Bluetooth_Serial.println(F("Bluetooth Active"));
#endif

}


