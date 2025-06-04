#pragma once

#include <Arduino.h>
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

#define TOTAL_TASK_TIME 10 * 1e3
#define RX_TIMEOUT_US 4000
// Adjusted based on the configured LoRa parameters (airtime) 
// This tx time has to fit in whatever time is left in the task after receiving
#define TX_TIMEOUT_US 2200

// Adjusted based on the configured LoRa parameters (airtime) 
#define APPROX_RX_TIME_US 2200
// Allow for some buffer time where the reception window is open, but we don't expect a packet
#define BUFFER_US 500
#define TARGET_RX_WAIT (APPROX_RX_TIME_US+BUFFER_US)
// Maximum step to slide the reception window
#define MAX_APPROX_SLIDE_STEP_US 50

#define DISCONNECT_TIMEOUT_US 250 * 1e3

#define RX_WAIT 0
#define TX_WAIT 1

#define RX_IRQ_MASK IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
#define TX_IRQ_MASK IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT

#define THROTTLE_MASK 0xFFF
#define TM_REQUEST_MASK 0x1000

extern SX128XLT LT;

TaskResult receiveThrottlePacket(unsigned long now);