/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 19/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


//*******  Setup hardware pin definitions here ! ***************


//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitiosn to match your own setup. Some pins such as DIO2,
//DIO3, may not be in used by this sketch so they do not need to be connected and
//should be set to -1.

const int8_t NSS = 7;                          //select on LoRa device
const int8_t NRESET = 8;                        //reset on LoRa device
const int8_t RFBUSY = 9;                        //RF busy on LoRa device 
const int8_t DIO1 = 6;                          //DIO1 on LoRa device, used for RX and TX done
const int8_t DIO2 = -1;                         //DIO2 on LoRa device, normally not used so set to -1
const int8_t DIO3 = -1;                         //DIO3 on LoRa device, normally not used so set to -1
const int8_t LED1 = 8;                          //On board LED, logic high is on
const int8_t RX_EN = -1;                        //pin for RX enable, used on some SX1280 devices, set to -1 if not used
const int8_t TX_EN = -1;                        //pin for TX enable, used on some SX1280 devices, set to -1 if not used

#define LORA_DEVICE DEVICE_SX1280               //this is the device we are using

const int8_t ppmPin = 2;                    //pin for controlling servo X1

const uint16_t RXIdentity = 123;                //define a receiver number, the transmitter must use the same number
                                                //range is 0 to 255

//*******  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
#define Frequency 2445000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes  
#define Bandwidth LORA_BW_0400                   //LoRa bandwidth
#define SpreadingFactor LORA_SF7                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate

#ifdef LTspeedMaximum 
#undef LTspeedMaximum 
#define LTspeedMaximum 1000000
#endif 

const uint8_t PacketLength = 3;                  //packet length is fixed
