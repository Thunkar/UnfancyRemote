#pragma once
//*******  Setup hardware pin definitions here ! ***************

const int NSS = 7;                           //select on LoRa device
const int NRESET = 9;                        //reset on LoRa device
const int RFBUSY = 10;                        //RF busy on LoRa device 
const int DIO1 = -1;                          //DIO1 on LoRa device, used for RX and TX done
const int DIO2 = -1;                         //DIO2 on LoRa device, normally not used so set to -1
const int DIO3 = -1;                         //DIO3 on LoRa device, normally not used so set to -1
const int RX_EN = -1;                        //pin for RX enable, used on some SX1280 devices, set to -1 if not used
const int TX_EN = -1;                        //pin for TX enable, used on some SX1280 devices, set to -1 if not used

const int BUTTON = 3;
const int PPM = 0;
const int LED = 8;
const int VBAT = 1;   

//*******  Setup constants here ! ***************

const unsigned long R2 = 6800;
const unsigned long R1 = 100000;

const unsigned int ENCODED_MAX = 4095;
const unsigned int ENCODED_HALF = 2048;

const unsigned int ADC_SAMPLES = 5;

// Task scheduling

const unsigned long periods[] = { 10, 20, 1000, 2000, 50 };
extern unsigned long lastRun[];
extern unsigned long executions[];

// Time in ms to hold the button to enter setup mode

const int setupModeDelay = 5000; 


//*******  Setup LoRa Test Parameters Here ! ***************

#define CH_BANDWIDTH_HZ 2500000
#define BASE_FREQUENCY 2400000000
#define LORA_DEVICE DEVICE_SX1280    
#define Offset 0                                 //offset frequency for calibration purposes  
#define Bandwidth LORA_BW_1600                   //LoRa bandwidth
#define SpreadingFactor LORA_SF5                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                    //LoRa coding rate

const unsigned int throttlePacketLength = 3;                  //packet length is fixed 
const unsigned int TMPacketLength = 2;
const unsigned int TXpower = 10;                       //LoRa transmit power in dBm
