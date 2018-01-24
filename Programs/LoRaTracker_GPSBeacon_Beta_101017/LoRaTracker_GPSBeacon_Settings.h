//LoRaTracker_GPSBeacon_Settings.h
/*
******************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 16/10/2017

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  To Do:

  Changes:
  101017 Removed unused board definitions

******************************************************************************************************
*/

//**************************************************************************************************
// 1) Hardware related definitions and options - specify board type here
//**************************************************************************************************

#define CPU_VoltageRead
//#define External_VoltageRead

#define CPU_TemperatureRead
//#define External_TemperatureRead
const int Temperature_Adjust = 0;                    //to allow calibration of external temperature sensor

#define Board_Definition "HAB2_Board_Definitions.h"  //define the board to use here       



//**************************************************************************************************
// 2) Program Options
//**************************************************************************************************


#define ConfigureDefaults                    //Configure settings from default program constants, save in memory and copy to RAM, need to do this once only
//#define ClearAllMemory                     //Clears from start memory to end memory, normally 1kbyte, needs to be folloowed by ConfigureDefaults
//#define ClearSavedData                     //zero the saved data, resets, sequence, Mahr
//#define ConfigureFromMemory                //Read settings from attached memory

#define CheckTone                            //comment in to have a calibrate tone at startup, also indicates navigation model 6 set

//#define DEBUG                              //if defined, prints additional debug information to terminal


//**************************************************************************************************
// 3) Frequency settings
//**************************************************************************************************

//Tracker mode
const uint32_t TrackerMode_Frequency = 434400000;

//Command mode
const uint32_t CommandMode_Frequency = 434400000;

//Bind mode - Change this with great care !!!!!
const uint32_t BindMode_Frequency = 434100000;

//this is the LoRa module frequency calibration offset in Hertz
const int CalibrationOffset = 0;


//**************************************************************************************************
// 4) LoRa settings
//**************************************************************************************************

//Tracker mode
#define TrackerMode_Bandwidth BW62500
#define TrackerMode_SpreadingFactor SF8
#define TrackerMode_CodeRate CR45
#define TrackerMode_Power 10

//Command mode
#define CommandMode_Bandwidth BW62500
#define CommandMode_SpreadingFactor SF8
#define CommandMode_CodeRate CR45
#define CommandMode_Power 10

//Bind mode
#define BindMode_Bandwidth BW500000
#define BindMode_SpreadingFactor SF8
#define BindMode_CodeRate CR45
#define BindMode_Power 2

#define SendBind                                //at startup tracker transmitter will send a bind packet
//#define LORADEBUG                             //displays extra debug messages when using LoRa
#define Accept_Commands                         //define this if you want to be able to remote control beacon


const uint8_t Deviation = 0x52;                 //typical deviation for tones
const uint8_t lora_RXBUFF_Size = 128;
const uint8_t lora_TXBUFF_Size = 32;

const uint16_t inter_Packet_delay = 500;        //allows time for receiver to be ready to see a possible reply, in mS


const uint8_t Cmd_WaitSecs = 5;                 //number of seconds to stay in waiting for command mode
const uint8_t Command_Loops = 3;                //if one command is received wait for this number of loops to keep control channel open

//Key Definition
const char key0 = 'L';                         //Used to restrict access to some commands
const char key1 = 'o';
const char key2 = 'R';
const char key3 = 'a';

//**************************************************************************************************
// 5) GPS Options
//**************************************************************************************************

#define USE_SOFTSERIAL_GPS                       //need to include this if we are using softserial for GPS     
#define GPS_Library "UBLOX_SerialGPS2.h"          //define the GPS library routines to use here, this is a version2 of the serial program, does not use Flash5 library
//#define GPS_Library "UBLOX_I2CGPS.h"          //define for using a UBLOX GPS with I2C interface

//#define DEBUGNoGPS                               //test mode, does not use GPS
//#define Use_Test_Location                        //to use test location for transmissions include this define

//#define GPS_ALLOW_GPGSV                        //define this so that GPGSV senetences are not turned off 

#define GPSBaud 9600                             //GPS Baud rate   
#define WhenNoGPSFix LeaveOn                     //What to do with GPS power when there is no fix at ends of wait period (LeaveOn or LeaveOff)
#define WaitGPSFixSeconds 30                     //in flight mode time to wait for a new GPS fix 
//#define Remove_GPS_Power                       //Some tracker boards can remove the power from the GPS, if so define this to use it
#define Use_GPS_SoftwareBackup                   //some GPSs do not support this mode, Ubloxes do, used for GPS Hotfix option

const uint32_t fixisoldmS = 10000;               //if location has not updated in this number of mS, assume GPS has lost fix
const uint32_t GPS_WaitAck_mS = 2000;            //number of mS to wait for an ACK response from GPS
const uint8_t GPS_attempts = 3;                  //number of times the sending of GPS config will be attempted.
const uint8_t GPS_Reply_Size = 12;               //size of GPS reply buffer
const unsigned int GPS_Clear_DelaymS = 2000;     //mS to wait after a GPS Clear command is sent  

//Centre of Cardiff Castle keep
#define TestLatitude 51.48230
#define TestLongitude -3.18136
#define TestAltitude 48


/*
  Passive GPS station: Swanbridge - C1ST1667
  https://www.ordnancesurvey.co.uk/gps/legacy-control-information/C1ST1667
  N 51 23'59.105955", W 3 11',47.413031
  51.399751654166664, -3.196503619722222

  #define TestLatitude 51.399752
  #define TestLongitude -3.196504
  #define TestAltitude 0
*/

//**************************************************************************************************
// 6) Which Memory to use for storage
//**************************************************************************************************

//#define Memory_Library "EEPROM_Memory.h"              //define this file if the internal EEPROM is in use
#define Memory_Library "I2CFRAM_MB85RC16PNF.h"          //define this file if the I2C FRAM is in use


//****************************************************************************************************
// 11) Program Default Option settings
//    This section determines which options are on or off by default, this is the Default_config1 byte
//    Take care here..........
//**************************************************************************************************

#define OptionOff 0
#define OptionOn 1

const char option_TXEnable = OptionOn;
const char option_AddressStrip = OptionOff;
const char option_GPSHotFix = OptionOff;

#define option_TXEnable_SUM (option_TXEnable*2)
#define option_AddressStrip_SUM (option_AddressStrip*64)
#define option_GPSHotFix_SUM (option_GPSHotFix*128)


const uint8_t Default_config1 = (option_TXEnable_SUM + option_AddressStrip_SUM + option_GPSHotFix_SUM);
//const uint16_t Default_config1 = 2;         //Phew, the default config can always be set manually .............


//**************************************************************************************************
// 13) Unique calibration settings for each Board
//    These settings are not part of a bind operation
//**************************************************************************************************

const float kelvin_offset = 325;                 //not used in this application but causes a complie error if missing
const float temp_conversion_slope = 1.0;         //not used in this application but causes a complie error if missing
uint32_t adc_constant = 1200000;                 //if processor self read of its supply voltage reports high reduce this number
     
//**************************************************************************************************
// 15) Beacon settings
//**************************************************************************************************


const uint16_t Loop_Sleepsecs = 10;                     //sleep time in seconds between transmissions
const boolean  promiscuous_Mode = true;                 //if set to True remote control packets from any node accepted

#define RemoteControlNode 'G'                           //normally used by tracker transmitter in promiscuous_Mode
#define ControlledNode '1'                              //normally used by tracker transmitter in promiscuous_Mode
#define ThisNode ControlledNode

