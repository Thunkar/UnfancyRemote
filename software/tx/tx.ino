/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 19/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a remote control transmitter that uses a LoRa link to transmit the positions
  from a simple joystick to a remote receiver. The receiver uses the sent joystick positions to adjust the
  positions of servos. The postions of the joysticks potentiometers on the transmitter are read with the
  analogueRead() function.

  If the joystick has a switch, often made by pressing on the joystick, then this can be used to remote
  control an output on the receiver. The switch is read by an interrupt, the interrupt routine sets a flag
  byte which is read in loop().

  The program is intended as a proof of concept demonstration of how to remote control servos, the program
  is not designed as a practical remote control device for RC model cars for instance.

  To have the transmitter program print out the values read from the joystick, comment in the line;

  //#define DEBUG

  Which is just above the loop() function. With the DEBUG enabled the transmission rate, the rate at which
  the control packets are transmitted will be slowed down.

  To reduce the risk of the receiver picking up LoRa packets from other sources, the packet sent contains a
  'TXidentity' number, valid values are 0 - 65535. The receiver must be setup with the matching identity
  number or the received packets will be ignored.

  The pin definitions, LoRa frequency and LoRa modem settings are in the Settings.h file. These settings
  are not necessarily optimised for long range.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>
#include <SX128XLT.h>
#include "settings.h"
#include <ProgramLT_Definitions.h>

SX128XLT LT;

uint8_t TXPacketL;

uint16_t throttleValue;                    //variable to read the value from the analog pin

int calMax = 500;
int calMin = 270;

#define DEBUG                              //comment in thie line (remove the two // at the beggining) for debug output
#define SX128XDEBUG
#define SX128XDEBUGPINS

void loop()
{
  throttleValue = analogRead(throttlePin);

  if (!sendThrottlePacket(throttleValue))
  {
    Serial.print(F("Send Error - IRQreg,"));
    Serial.print(LT.readIrqStatus(), HEX);
  }
}


uint8_t sendThrottlePacket(uint16_t throttleValue)
{
  //The SX12XX buffer is filled with variables of a known type and in a known sequence. Make sure the
  //receiver uses the same variable types and sequence to read variables out of the receive buffer.

  int scaledValue = map(throttleValue, calMin, calMax, 0, 254);
  scaledValue = constrain(scaledValue, 0, 254);
  byte encodedValue = 254 - (byte)scaledValue;
  
  LT.startWriteSXBuffer(0);                      //start the write packet to buffer process
  LT.writeUint8(RControl1);                      //this is the packet type
  LT.writeUint8(TXIdentity);                     //this value represents the transmitter number
  LT.writeUint8(encodedValue);                        //this byte contains joystick pot AD X1 value to be sent
  LT.endWriteSXBuffer();                         //close the packet, thee are 5 bytes to send

  //now transmit the packet, 10 second timeout, and wait for it to complete sending
  TXPacketL = LT.transmitSXBuffer(0, PacketLength, 10000, TXpower, WAIT_TX);

#ifdef DEBUG
  Serial.print(TXIdentity);
  Serial.print(F(",Raw,"));
  Serial.print(throttleValue);
  Serial.print(F(",Scaled,"));
  Serial.print(scaledValue);
  Serial.print(F(",Encoded,"));
  Serial.print(encodedValue);
  Serial.println();
#endif

  return TXPacketL;                                //TXPacketL will be 0 if there was an error sending
}


void setup()
{

  Serial.begin(115200);

  SPI.begin();
  

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("Init complete"));
  }
  else
  {
    Serial.println(F("Device error"));
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  Serial.println(F("35_Remote_Control_Servo_Transmitter ready"));
}
