#pragma once

#include <SX128XLT.h>
#include <ProgramLT_Definitions.h>
#include "board.h"
#include "utils.h"
#include "state.h"
#include "config.h"
#include "error_handling.h"	
#include "stats.h"

#define RX_TIMEOUT_US 15 * 1e3
#define TX_TIMEOUT_US 5 * 1e3
#define RECEPTION_TIME_TARGET_US RX_TIMEOUT_US / 2

#define DISCONNECT_TIMEOUT_US 200 * 1e3

#define RX_WAIT 0
#define TX_WAIT 1

#define RX_IRQ_MASK IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
#define TX_IRQ_MASK IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT

#define THROTTLE_MASK 0xFFF
#define TM_REQUEST_MASK 0x1000

extern SX128XLT LT;

TaskResult receiveThrottlePacket(unsigned long now);