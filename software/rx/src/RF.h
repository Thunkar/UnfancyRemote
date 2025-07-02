#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SX128XLT.h>
#include <ProgramLT_Definitions.h>
#include "board.h"
#include "utils.h"
#include "state.h"
#include "config.h"
#include "stats.h"

// Lora params

#define LORA_DEVICE DEVICE_SX1280    
#define Offset 0                               
#define Bandwidth LORA_BW_1600                  
#define SpreadingFactor LORA_SF7                 
#define CodeRate LORA_CR_LI_4_8   
#define PREAMBLE_LENGTH 8
#define TX_POWER 12                      
#define THROTTLE_PACKET_LENGTH 3                 
#define TM_PACKET_LENGTH 2

#define TX_TIMEOUT_US (3 * 1e3)
#define RX_TIMEOUT_US (10 * 1e3)

#define DISCONNECT_TIMEOUT_US 250 * 1e3

#define RX_WAIT 0
#define TX_WAIT 1

#define RX_IRQ_MASK IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
#define TX_IRQ_MASK IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT

#define THROTTLE_MASK 0xFFF
#define TM_REQUEST_MASK 0x1000

extern SX128XLT LT;

enum RF_STATE {
    IDLE,
    RX_WAITING,
    RX_DONE,
    TX_WAITING,
};

TaskResult receiveThrottlePacket(unsigned long now);
TaskResult checkRFStatus(unsigned long now);

void setupLoRa();