/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 19/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a remote control receiver that uses a LoRa link to control the positions of
  servos sent from a remote transmitter.

  If the transmitter joystick has a switch, often made by pressing on the joystick, then this can be used
  to remote control an output on the receiver.

  The program is intended as a proof of concept demonstration of how to remote control servos, the program
  is not designed as a practical remote control device for RC model cars for instance.

  To have the receiver program print out the joystick values (0-255) read from the received packet, comment
  in the line;

  //#define DEBUG

  Which is just above the loop() function. With the DEBUG enabled then there is a possibility that some
  transmitted packets will be missed. With the DEBUG line enabled to servos should also sweep to and fro 3
  times at program start-up.

  To reduce the risk of the receiver picking up LoRa packets from other sources, the packet sent contains a
  'TXidentity' number, valid values are 0 - 255. The receiver must be setup with the matching RXIdentity
  number in Settings.h or the received packets will be ignored.

  The pin definitions, LoRa frequency and LoRa modem settings are in the Settings.h file. These settings
  are not necessarily optimised for long range.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#define programversion "V1.0"

#include <SPI.h>
#include <SX128XLT.h>
#include "settings.h"
#include <ProgramLT_Definitions.h>

SX128XLT LT;

#include <Servo.h>
Servo PPM;                             //create the servo object

uint8_t throttleValue;                   //variable to read the value from the analog pin
uint8_t RXPacketL;                         //length of received packet
uint8_t RXPacketType;                      //type of received packet

#define DEBUG


void loop()
{
  uint16_t IRQStatus;
  
  RXPacketL = LT.receiveSXBuffer(0, 0, WAIT_RX);   //returns 0 if packet error of some sort

  while (!digitalRead(DIO1)){
    Serial.println(F("no signal"));                      //wait for DIO1 to go high
  }
  
  if  (LT.readIrqStatus() == (IRQ_RX_DONE + IRQ_HEADER_VALID + IRQ_PREAMBLE_DETECTED) )
  {
    packet_is_OK();
  }
  else
  {
    packet_is_Error();
  }

}


uint8_t packet_is_OK()
{
  //packet has been received, now read from the SX12xx Buffer using the same variable type and
  //order as the transmit side used.
  uint8_t TXIdentity;
  uint16_t throttlePulse;

  LT.startReadSXBuffer(0);                //start buffer read at location 0
  RXPacketType = LT.readUint8();          //read in the packet type
  TXIdentity = LT.readUint8();            //read in the transmitter number
  throttleValue = LT.readUint8();       
  RXPacketL = LT.endReadSXBuffer();       //end buffer read


  if (RXPacketType != RControl1)
  {
    Serial.print(F("Packet type "));
    Serial.println(RXPacketType);
    return 0;
  }


  if (TXIdentity != RXIdentity)
  {
    Serial.print(F("TX"));
    Serial.print(TXIdentity);
    Serial.println(F("?"));
    return 0;
  }

  throttlePulse = map(throttleValue, 0, 254, 1000, 2000);

  Serial.print(TXIdentity);
  Serial.print(F(",Throttle,"));
  Serial.print(throttleValue);
  Serial.print(F(",PPM,"));
  Serial.print(throttlePulse);
  Serial.println();
  
  PPM.writeMicroseconds(throttlePulse);

  return RXPacketL;
}


void packet_is_Error()
{
  uint16_t IRQStatus;
  int16_t PacketRSSI;
  IRQStatus = LT.readIrqStatus();

  if (IRQStatus & IRQ_RX_TIMEOUT)
  {
    Serial.print(F("RXTimeout"));
  }
  else
  {
    PacketRSSI = LT.readPacketRSSI();                        //read the signal strength of the received packet
    Serial.print(F("Err,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm"));
  }
  Serial.println();
}


void setup()
{
  Serial.begin(115200);
    
  SPI.begin();

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("LoRa ready"));
  }
  else
  {
    Serial.println(F("Device error"));
  }

  PPM.attach(ppmPin);

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  Serial.println(F("Rx ready"));
  Serial.println();
}
