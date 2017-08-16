//HAB2_Settings.h
/*
******************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 15/08/2017

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  To Do:

******************************************************************************************************
*/

//**************************************************************************************************
// 1) Hardware related definitions and options - specify board type here
//**************************************************************************************************

//#define External_VoltageRead
#define CPU_VoltageRead

//#define External_TemperatureRead
#define CPU_TemperatureRead

#define Board_Definition "HAB2_Board_Definitions.h"
//#define "PIHTracker3_Board_Definitions.h"

const float runmA = 4;                        //processor run current
const float SleepmA = 0.22;                   //approx current in sleep, GPS consumes circa 200uA


//**************************************************************************************************
// 2) Program Options
//**************************************************************************************************


//#define ClearSavedData                     //zero the saved data, resets, sequence, Mahr
//#define ClearAllMemory                     //Clears from start memory to end memory, normally 1kbyte, needs to be folloowed by ConfigureDefaults
#define ConfigureDefaults                    //Configure settings from default program constants, save in memory and copy to RAM, do this once only
//#define ConfigureFromMemory                //Read settings from attached memory

//#define write_CalibrationOffset            //comment in to write calibration constant to memory, needs to be done once only.

const byte Output_len_max = 125;             //maximum length for built payload
#define CheckTone                            //comment in to have a calibrate tone at startup, also doubles as GPS check tone


//#define DEBUG                              /if defined, prints additional debug information to terminal

const byte Cmd_WaitSecs = 5;                 //number of seconds to stay in command mode
const byte default_attempts = 5;             //default number of times a command will attempt to be sent
const int DozeSleepSecs = 60;                //how many seconds to spend in doze (very low power mode), can only be set remotely


//**************************************************************************************************
// 3) Frequency settings
//**************************************************************************************************

//Tracker mode
const unsigned long TrackerMode_Frequency = 434400000;

//Search mode
const unsigned long SearchMode_Frequency = 434400000;

//Command mode
const unsigned long CommandMode_Frequency = 434500000;

//Bind mode - Change this with great care !!!!!
const unsigned long BindMode_Frequency = 434100000;

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

//Search mode
#define SearchMode_Bandwidth BW62500
#define SearchMode_SpreadingFactor SF12
#define SearchMode_CodeRate CR45
#define SearchMode_Power 10

//Command mode
#define CommandMode_Bandwidth BW62500
#define CommandMode_SpreadingFactor SF10
#define CommandMode_CodeRate CR45
#define CommandMode_Power 5

//Bind mode
#define BindMode_Bandwidth BW500000
#define BindMode_SpreadingFactor SF8
#define BindMode_CodeRate CR45
#define BindMode_Power 2

#define SendBind                             //at startup tracker transmitter will send a bind packet
//#define ReceiveBind                        //during flight allows for a bind to be received
//#define LORADEBUG                          //displays extra debug messages when using LoRa

const byte Deviation = 0x52;                 //typical deviation for tones
const byte lora_RXBUFF_Size = 128;
const byte lora_TXBUFF_Size = 128;

const float RXmA = 11;                       //LoRa device receive current
const float TXmA = 40;                       //LoRa device transmit current @ 10dBm

const int inter_Packet_delay = 500;          //allows time for receiver to be ready to see a possible reply
const byte delayforRelaysecs = 2;            //allows time for relay to re-transmit
const byte Command_Loops = 3;                //if one command is received wait for this number of loops to keep control channel open

const char key0 = 'L';                       //Used to restrict access to some commands
const char key1 = 'o';
const char key2 = 'R';
const char key3 = 'a';

//**************************************************************************************************
// 5) GPS Options
//**************************************************************************************************

#define USE_SOFTSERIAL_GPS                       //need to include this if we are using softserial for GPS     
//#define GPS_Library "UBLOX_SerialGPS.h"          //define the GPS library routines to use here
#define GPS_Library "UBLOX_I2CGPS.h"

//#define DEBUGNoGPS                             //test mode, does not use GPS

 
#define WhenNoGPSFix LeaveOn                     //What to do with GPS power when there is no fix at ends of wait period (LeaveOn or LeaveOff)
#define GPSPowerControl Enabled                  //Some tracker boards can remove the power form the GPS
#define Use_GPS_SoftwareBackup                   //some GPSs do not support this mode
//#define GPS_ALLOW_GPGSV                        //comment in to prevent GPGSV sentances being turned off on a UBLOX GPS

const unsigned int GPSBaud = 9600;               //GPS Baud rate
const byte WaitGPSFixSeconds = 10;               //in flight mode time to wait for a new GPS fix 
const unsigned long GPSShutdownTimemS = 1900;    //Software backup mode takes around 1.9secs to power down, used in maHr calculation
const unsigned long fixisoldmS = 10000;          //if location has not updated in this number of mS, assume GPS has lost fix
const unsigned long GPS_WaitAck_mS = 2000;       //number of mS to wait for an ACK response from GPS
const unsigned int GPSFixs = 100;                //number of GPS fixes between setting system clock from GPS
const byte GPS_attempts = 3;                     //number of times the sending of GPS config will be attempted.
const byte GPS_Reply_Size = 12;                  //size of GPS reply buffer
const float GPSmA = 24;                          //GPS average (UBLOX) current when aquiring fix


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

#define Memory_Library "EEPROM_Memory.h"


//**************************************************************************************************
// 8) FSK RTTY Settings
//**************************************************************************************************

const unsigned int FSKRTTYbaudDelay = 9850;         //delay for baud rate for FSK RTTY, 19930 for 50baud, 9900 for 100baud, 4930 for 200baud
const byte FSKRTTYRegshift = 6;                     //number of frequency steps to use for shift
const byte FSKRTTYpips = 5;                         //number of FSK lead in pips
const int  FSKRTTYleadin = 500;                     //number of ms for FSK constant lead in tone
const byte sync_chars = 3;                          //number of extra $ sync characters to send


//**************************************************************************************************
// 9) AFSK RTTY Options
//**************************************************************************************************

const byte delayforAFSKuploadSecs = 4;              //allows time for AFSK upload on receiver


//****************************************************************************************************
// 11) Program Default Option settings
//    This section determins which options are on or off by default, this is the default_config byte
//    Take care here..........
//**************************************************************************************************

#define OptionOff 0
#define OptionOn 1

const char option_SearchEnable = OptionOn;
const char option_TXEnable = OptionOn;
const char option_FSKRTTYEnable = OptionOn;
const char option_CheckFence = OptionOn;
const char option_ShortPayloadEnable = OptionOff;
const char option_RepeatEnable = OptionOff;
const char option_AddressStrip = OptionOff;
const char option_GPSPowerSave = OptionOn;

#define option_SearchEnable_SUM (option_SearchEnable*1)
#define option_TXEnable_SUM (option_TXEnable*2)
#define option_FSKRTTYEnable_SUM (option_FSKRTTYEnable*4)
#define option_CheckFence_SUM (option_CheckFence*8)
#define option_ShortPayloadEnable_SUM (option_ShortPayloadEnable*16)
#define option_RepeatEnable_SUM (option_RepeatEnable*32)
#define option_AddressStrip_SUM (option_AddressStrip*64)
#define option_GPSPowerSave_SUM (option_GPSPowerSave*128)

const unsigned int Default_config1 = (option_SearchEnable_SUM + option_TXEnable_SUM + option_FSKRTTYEnable_SUM + option_CheckFence_SUM + option_ShortPayloadEnable_SUM + option_RepeatEnable_SUM + option_AddressStrip_SUM + option_GPSPowerSave_SUM);
const unsigned int Default_config2 = 0;
const unsigned int Default_config3 = 0;
const unsigned int Default_config4 = 0;
//const unsigned int const_Default_config = 196;         //Phew, the default config can always be set manually .............


//**************************************************************************************************
// 12) Miscellaneous program settings
//**************************************************************************************************





//**************************************************************************************************
// 13) Unique calibration settings for each Board
//    These settings are not part of a bind operation
//**************************************************************************************************

const float kelvin_offset = 320;                  //if processor self read of temperature reports high, increase this number
const float temp_conversion_slope = 1.27;         //defines the rate of change between low and high temperatures
const unsigned long  adc_constant = 1146679;      //if processor self read of its supply voltage reports high reduce this number


//**************************************************************************************************
// 14) HAB2 settings
//**************************************************************************************************

char Flight_ID[15] = "MYHAB";
const float west_fence = -4;
const float east_fence = 45;

const unsigned int Loop_Sleepsecs = 25;                 //sleep time in seconds after each TX loop
const unsigned int outside_fence_Sleep_seconds = 600;   //approx 10 minutes
const boolean  promiscuous_Mode = true;                 //if set to True remote control packets from any node accepted

#define RemoteControlNode 'G'                           //normally used by tracker transmitter in promiscuous_Mode
#define ControlledNode '1'                              //normally used by tracker transmitter in promiscuous_Mode
#define ThisNode ControlledNode

const byte PayloadArraySize = 20;                       //Maximum number of fields when parsing long HAB payload


//**************************************************************************************************
// 15) Locator2 Settings
//**************************************************************************************************

//#define RCPulseMode                                   //select if RC servo pulse reading to be used for lost detection
#define Timeout
const int MinsToLost = 40;                              //minutes before lost mode automatically engaged.
const int TXDelaySecs = 15;                             //delay in seconds between position transmissions, needed to ensure duty cycle limits kept, normally 10%
const int TXLostDelaySecs = 15;                         //delay in seconds between lost mode transmissions


//Program constants
const int pulseerrorlimit = 100;                        //number of RC error pulses needed to trigger lost condition, 255 max
const int holddifference = 30;                          //if differance between two RC pulses is less than this, then hold is assumed
const int RCpulseshort = 750;                           //in uS, lower than this RC pulse is assumed not to be valid
const int RCpulselong = 2500;                           //in uS, higher than this RC pulse is assumed not to be valid
const unsigned long GPSerrorLimit = 129000;             //number of times around loop with no characters before an error is flagged, around 5seconds
const byte inc_on_error = 5;                            //number to increase pulse error count on a fail
const byte dec_on_OK = 10;                              //number to decrease pulse error count on a pass

