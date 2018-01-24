//Basic_Receiver_Terminal_Settings.h
/*
******************************************************************************************************

LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson

http://www.LoRaTracker.uk
  
These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

Note that if you select all possible options, SD Card logging, Bluetooth NMEA uplink and AFSK RTTY
upload you will run out of memory whne the ATMEGA328 is in use, cannot do much about it.

To Do:
disable bind mode when menu not selected

******************************************************************************************************
*/

//**************************************************************************************************
// 1) Hardware related definitions and options - specify board type here
//**************************************************************************************************

//#define UseSD                                            //define if there is an SD card attached 
#define Board_Definition "LCD_Receiver_Board_Definitions.h"    //define the board type to be used


//**************************************************************************************************
// 2) Program Options
//**************************************************************************************************

//#define ClearConfigData                 //at startup zero the config memory 
//#define ClearAllMemory                  //at startup clears from start memory to end memory, normally 1kbyte, needs to be followed by ConfigureDefaults
#define ConfigureDefaults                 //at startup configure settings from default program defaults, these are then stored in memory
//#define ConfigureFromMemory               //at startup configure settings from values stored in memory, this needs to be the active mode for bind to work

#define CalibrateTone                     //comment in to have a calibrate tone at startup



//#define NoMenu

//**************************************************************************************************
// 3) Frequency settings
//**************************************************************************************************

//Tracker mode
const unsigned long TrackerMode_Frequency = 434400000;   

//Search mode
const unsigned long SearchMode_Frequency = 434300000;   

//Command mode
const unsigned long CommandMode_Frequency = 434500000;   

//Bind mode - Change this with great care !!!!!
const unsigned long BindMode_Frequency = 434100000;      

//this is the LoRa module frequency calibration offset in Hertz
const int CalibrationOffset = 0;

//**************************************************************************************************
// 4) LoRa settings
//**************************************************************************************************

#define LoRa_Device_in_MB1                  //if using a MikroBus based specify the socket for the LoRa Device 

//#define LORADEBUG                         //if defined prints some LoRa debug information to serial monitor
#define LORA_AFC                            //define this if using the afc function

//Tracker mode
const byte TrackerMode_Power = 10;
#define TrackerMode_Bandwidth BW7800
#define TrackerMode_SpreadingFactor SF8
#define TrackerMode_CodeRate CR45

//Search mode
const byte SearchMode_Power = 10;
#define SearchMode_Bandwidth BW62500
#define SearchMode_SpreadingFactor SF12
#define SearchMode_CodeRate CR45

//Command mode
const byte CommandMode_Power = 5;
#define CommandMode_Bandwidth BW62500
#define CommandMode_SpreadingFactor SF10
#define CommandMode_CodeRate CR45


//Bind mode - Change this with great care !!!!!
const byte BindMode_Power = 2;
#define BindMode_Bandwidth BW500000
#define BindMode_SpreadingFactor SF8
#define BindMode_CodeRate CR45

const byte Deviation = 0x52;    //typical deviation for tones
const byte lora_RXBUFF_Size = 128;
const byte lora_TXBUFF_Size = 128;


#define RemoteControlNode 'G'                //normally used by tracker transmitter in promiscuous_Mode
#define ControlledNode '1'                   //normally used by tracker transmitter in promiscuous_Mode
#define ThisNode ControlledNode
const boolean  Promiscuous_Mode = true;      //if set to false remote control packets from any node accepted
const int inter_Packet_delay = 500;          //allows time for receiver to be ready to see a possible reply
const byte Cmd_WaitSecs = 15;                //number of seconds to stay in command mode  
const byte default_attempts = 5;             //default number of times a command will attempt to be sent


//Protected Command Settings
const char key0 = 'L';                       //Used to restrict access to some commands
const char key1 = 'o';
const char key2 = 'R';
const char key3 = 'a';

//#define No_DIO0_RX_Ready                   //needed for boards that cannot directly ready the DIO0 pin 

 

//**************************************************************************************************
// 6) Which Memory to use for storage
//**************************************************************************************************

#define Memory_Library "I2CFRAM_MB85RC16PNF.h" 
//#define Memory_Library "EEPROM_Memory.h"
//#define Memory_Library "SPIFRAM_Memory_FM25CL64B_MB85RS64.h"       //define this file if a SPI FRAM is in use


//**************************************************************************************************
// 9) AFSK RTTY Options
//**************************************************************************************************

//#define USE_AFSK_RTTY_Upload
const int AFSKrttybaud = 1465;            //delay in uS x 2 for 1 bit and 300baud. Decode range in FLDIGI 1420 to 1510  
const int afskleadinmS = 500;             //number of ms for AFSK constant lead in tone
const int tonehighHz = 1000;              //high tone in Hertz 
const int tonelowHz = 650;                //low tone in Hertz   


//**************************************************************************************************
// 10) Bluetooth Options
//**************************************************************************************************

//#define Use_NMEA_Bluetooth_Uplink        //define if your going to use a Bluetooth adapter to uplink NMEA into a mapping application  
#define USE_SendOnlySoftwareSerial       //if you want the Bluetooth device on a pin other than the hardware serial port, define this
#define USE_HardwareSerial               //use hardware serial port to send Bluetooth
#define Hardware_Bluetooth_Port Serial   //Which Hardware port to use for Bluetooth
#define BluetoothBaud 9600 

const byte Bluetooth_Buff_Size = 128;    //size of buffer for NMEA output


//**************************************************************************************************
// 12) Miscellaneous program settings
//**************************************************************************************************

const unsigned int switch_delay = 1000;  //delay in mS after a switch\key press before another can take place

