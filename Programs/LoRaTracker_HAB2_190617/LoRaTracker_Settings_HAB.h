//LoRaTracker_Settings_HAB.h
/*
******************************************************************************************************

LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson - 19/06/2017

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
// 1a) Hardware realted definitions and options - you must specify the board in use here.
//     Details of all the boards that can be used are in the 'Program_Definitions.h' library file
//     
//     Note: Some programs will not run on some boards; the receiver program for instance needs a
//     board that has a SD card connected.
//**************************************************************************************************

#define LoRaTracker_HAB2             
//#define UseSD                                               //Select to use SD card for logging

#define External_VoltageRead
//#define CPU_VoltageRead

#define External_TemperatureRead
//#define CPU_TemperatureRead

//**********************************************
// 1b) GPS Options
//**********************************************

#define USE_SERIALGPS                          //comment in if using SoftwareSerial
                        
#define USE_SOFTSERIAL_GPS
//#define USE_I2CGPS   

#define UBLOX
#define GPSBaud 9600

#define WhenNoGPSFix LeaveOn                   //What to do with GPS power when there is no fix at ends of wait period (LeaveOn or LeaveOff)
#define WaitGPSFixSeconds 5                    //in flight mode time to wait for a new GPS fix 
#define GPSPowerControl Enabled                //Some tracker boards can remove the power form the GPS

#define DebugNoGPS                             //test mode, does not use GPS
#define TestLocation                           //uses test locations as defined in Flight_Settings

#define GPSShutdownTimemS 1900                 //Software backup mode takes around 1.9secs to power down, used in maHr calculation

#define fixisoldmS 10000                       //if location has not updated in this number of mS, assume GPS has lost fix
#define GPSFixs 100                            //number of GPS fixes between setting system clock from GPS   

//**************************************************************************************************
// 1c) Bluetooth Options
//**************************************************************************************************

//#define Bluetooth_Baud 9600 
//#define USE_NMEA_Bluetooth

//**************************************************************************************************
// 1d) Which memory to use for storage
//**************************************************************************************************

//#define USE_I2C_FRAM_MEMORY1                  //select if using i2c FRAM
//#define USE_SPI_FRAM_MEMORY1                //select if using SPI FRAM
#define USE_EEPROM_MEMORY
//#define USE_NoMemory                        //define if payloads not to be saved in memory

//**************************************************************************************************
// 1e) Module locations for MikroBus Shield Boards
//**************************************************************************************************

//#define LoRa_Device_in_MB1                  //defines which Mikrobus socket the LoRa device is in
//#define GPS_in_MB2                          //defines which Mikrobus socket the GPS is in
 

 //**************************************************************************************************
// 1f) AFSK RTTY Upload Options
//**************************************************************************************************

//#define USE_AFSK_RTTY_Upload
//const int AFSKrttybaud = 1465;             //delay in uS x 2 for 1 bit and 300baud. Decode range in FLDIGI 1420 to 1510  
//const int afskleadinmS = 500;              //number of ms for AFSK constant lead in tone
//const int tonehighHz = 1000;               //high tone in Hertz 
//const int tonelowHz = 650;                 //low tone in Hertz   

 
 
//**************************************************************************************************
// 2) Unique settings for each LoRaTracker board
//    These settings are not part of a bind operation
//**************************************************************************************************

const int CalibrationOffset = 0;              //this is the LoRa module frequency calibration offset in Hertz
const float kelvin_offset = 334;              //if processor self read of temperature reports high, increase this number
const float temp_conversion_slope = 1.27;     //defines the rate of change between low and high temperatures
const long  adc_constant = 1135194;           //if processor self read of its supply voltage reports high reduce this number 


//**************************************************************************************************
// 3) Program Options - comment in or out 
//**************************************************************************************************


//#define ClearSavedData                     //zero the saved data, resets, sequence, Mahr
#define ConfigureDefaults                    //Configure settings from default program constants, save in memory and copy to RAM, do this once only
//#define ConfigureFromMemory                //Read settings from attached memory

#define Output_len_max 125                   //maximum length for built payload
#define CalibrateTone                        //comment in to have a calibrate tone at startup

#define SendBind                             //at startup tracker transmitter will send a bind packet
#define write_CalibrationOffset              //comment in to write calibration constant to memory, needs to be done once only.

#define DEBUG

//#define ReceiveBind                        //during flight allows for a bind to be received    



//**************************************************************************************************
// 4)  Not used - Left for future applications
//**************************************************************************************************



//**************************************************************************************************
// 5) HAB flight settings
//**************************************************************************************************

char Flight_ID[15] = "LoRaTracker1";
const float west_fence = -32;
const float east_fence = 45;
#define Sleepsecs 5                             //sleep time in seconds after each TX loop
#define outside_fence_Sleep_seconds 600         //approx 10 minutes

#define RemoteControlNode 'G'                   //normally used by tracker transmitter in promiscuous_Mode
#define ControlledNode '1'                      //normally used by tracker transmitter in promiscuous_Mode
#define ThisNode ControlledNode
const boolean  promiscuous_Mode = true;         //if set to True remote control packets from any node accepted
//const boolean  promiscuous_Mode = false;      //if set to False remote control packets only accpted from 'remoteControl_Node'


//**************************************************************************************************
// 6) Option settings  These determine which options are on or off by default, this is the default_config byte
//**************************************************************************************************

#define OptionOff 0
#define OptionOn 1

const char option_SearchEnable = OptionOn;
const char option_TXEnable = OptionOn;
const char option_FSKRTTYEnable = OptionOff;        
const char option_CheckFence = OptionOff;           
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

const unsigned int Default_config1 = (option_SearchEnable_SUM+option_TXEnable_SUM+option_FSKRTTYEnable_SUM+option_CheckFence_SUM+option_ShortPayloadEnable_SUM+option_RepeatEnable_SUM+option_AddressStrip_SUM+option_GPSPowerSave_SUM);
const unsigned int Default_config2 = 0;
const unsigned int Default_config3 = 0;
const unsigned int Default_config4 = 0;
//const unsigned int const_Default_config = 196;         //Phew, the default config can always be set manually .............


//**************************************************************************************************
// 7) LoRa Frequency and Modem settings
//**************************************************************************************************


//Tracker mode
#define TrackerMode_Frequency 434400000
#define TrackerMode_Bandwidth BW62500
#define TrackerMode_SpreadingFactor SF8
#define TrackerMode_CodeRate CR45
#define TrackerMode_Power 10

//Search mode
#define SearchMode_Frequency 434400000
#define SearchMode_Bandwidth BW62500
#define SearchMode_SpreadingFactor SF12
#define SearchMode_CodeRate CR45
#define SearchMode_Power 10

//Command mode
#define CommandMode_Frequency 434500000
#define CommandMode_Bandwidth BW62500
#define CommandMode_SpreadingFactor SF10
#define CommandMode_CodeRate CR45
#define CommandMode_Power 5

//Bind mode
#define BindMode_Frequency 434100000
#define BindMode_Bandwidth BW500000
#define BindMode_SpreadingFactor SF8
#define BindMode_CodeRate CR45
#define BindMode_Power 2

const byte Deviation = 0x52;    //typical deviation for tones

//**************************************************************************************************
// 8) Various Program Settings
//**************************************************************************************************


const byte Cmd_WaitSecs = 5;                       //number of seconds to stay in command mode  
const byte default_attempts = 5;                    //default number of times a command will attempt to be sent
const int DozeSleepSecs = 60;                       //how many seconds to spend in doze (very low power mode), can only be set remotely 

#define PayloadArraySize 20                         //Maximum number of fields when parsing long HAB payload 
#define Portable_Mode 1                             //used as flag and return values 
#define Terminal_Mode 2                             //used as flag and return values
#define max_functions 5                             //number of functions in main program loop

const int inter_Packet_delay = 500;                 //allows time for receiver to be ready to see a possible reply
const byte delayforRelaysecs = 6;                   //allows time for relay to re-transmit

//**************************************************************************************************
// 9) FSK RTTY Settings
//**************************************************************************************************

const unsigned int FSKRTTYbaudDelay = 9900;         //delay for baud rate for FSK RTTY, 19930 for 50baud, 9900 for 100baud, 4930 for 200baud (100baud was 9800)  
const byte FSKRTTYRegshift = 6;                     //number of frequency steps to use for shift
const byte FSKRTTYpips = 5;                         //number of FSK lead in pips
const int  FSKRTTYleadin = 500;                     //number of ms for FSK constant lead in tone
 


//**************************************************************************************************
// 10) Protected Command Settings
//**************************************************************************************************

const char key0 = 'L';                              //Used to restrict access to some commands
const char key1 = 'o';
const char key2 = 'R';
const char key3 = 'a';


//**************************************************************************************************
// 11) GPS Test location settings - used for testing purposes
//**************************************************************************************************

/*
Centre of Cardiff Castle keep
*/

//#define TestLatitude 51.48230                       
//#define TestLongitude -3.18136
//#define TestAltitude 48

/*
Passive GPS station: Swanbridge - C1ST1667
https://www.ordnancesurvey.co.uk/gps/legacy-control-information/C1ST1667
N 51 23'59.105955", W 3 11',47.413031
51.399751654166664, -3.196503619722222  
*/

#define TestLatitude 51.399752                       
#define TestLongitude -3.196504
#define TestAltitude 0


//**************************************************************************************************
// 12) Display Settings 
//**************************************************************************************************

#define USE_Display_5110
const byte contrast5110 = 50;
byte tsize = 1;               //used to keep track of current text size 1 or 2


//**************************************************************************************************
// 13) NMEA Output Settings 
//**************************************************************************************************

#define Payload_buffer 128                            //size of buffer for NMEA output, matches that of lora_TXBUFF[128]


//**************************************************************************************************
// 14) Not used - Left for future applications
//**************************************************************************************************


//**************************************************************************************************
// 15) Lost Model Locator Settings
//**************************************************************************************************

//#define RCPulseMode                        //select if RC servo pulse reading to be used for lost detection 
#define Timeout
#define MinsToLost 40                        //minutes before lost mode automatically engaged.
#define TXDelaySecs 15                       //delay in seconds between position transmissions, needed to ensure duty cycle limits kept, normally 10%
#define TXLostDelaySecs 15                   //delay in seconds between lost mode transmissions
#define Output_len_max 126                   //max length of outgoing packet

//Program constants
const int pulseerrorlimit = 100;              //number of RC error pulses needed to trigger lost condition, 255 max
const int holddifference = 30;               //if differance between two RC pulses is less than this, then hold is assumed
const int RCpulseshort = 750;                //in uS, lower than this RC pulse is assumed not to be valid
const int RCpulselong = 2500;                //in uS, higher than this RC pulse is assumed not to be valid
const unsigned long GPSerrorLimit = 129000;  //number of times around loop with no characters before an error is flagged, around 5seconds
const byte inc_on_error = 5;                 //number to increase pulse error count on a fail
const byte dec_on_OK = 10;                   //number to decrease pulse error count on a pass

