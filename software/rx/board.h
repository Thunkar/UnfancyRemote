//*******  Setup hardware pin definitions here ! ***************
//#define HW_V1
#define HW_V2
const int NSS = 7;                           //select on LoRa device
const int NRESET = 8;                        //reset on LoRa device
const int RFBUSY = 9;                        //RF busy on LoRa device 
const int DIO1 = 6;                          //DIO1 on LoRa device, used for RX and TX done
const int DIO2 = -1;                         //DIO2 on LoRa device, normally not used so set to -1
const int DIO3 = -1;                         //DIO3 on LoRa device, normally not used so set to -1
const int RX_EN = -1;                        //pin for RX enable, used on some SX1280 devices, set to -1 if not used
const int TX_EN = -1;                        //pin for TX enable, used on some SX1280 devices, set to -1 if not used

const int BUTTON = A0;
const int PPM_L1 = 2;
const int L2 = 3;
const int L3 = 4;
const int L4 = 5;
const int VBAT = A1;   

#ifdef HW_V1
const unsigned long R2 = 7100;
#else
const unsigned long R2 = 6800;
#endif
const unsigned long R1 = 100000;


//*******  Setup LoRa Test Parameters Here ! ***************


#define LORA_DEVICE DEVICE_SX1280    
#define Offset 0                                 //offset frequency for calibration purposes  
#define Bandwidth LORA_BW_1600                   //LoRa bandwidth
#define SpreadingFactor LORA_SF7                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate
#define CH_BANDWIDTH_HZ 2500000
#define BASE_FREQUENCY 2400000000

const unsigned int throttlePacketLength = 4;                  //packet length is fixed 
const unsigned int TMPacketLength = 3;
const unsigned int TXpower = 10;                       //LoRa transmit power in dBm
