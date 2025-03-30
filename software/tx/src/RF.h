#pragma once

#include <SX128XLT.h>
#include <ProgramLT_Definitions.h>
#include "board.h"
#include "scheduler.h"
#include "utils.h"
#include "state.h"
#include "config.h"
#include "error_handling.h"	
#include "stats.h"


#define RX_TIMEOUT_US 5 * 1e3
#define TX_TIMEOUT_US 5 * 1e3

#define TM_PERIOD_US 500 * 1e3
#define TM_TIMEOUT_US 5000 * 1e3

#define RX_WAIT 0
#define TX_WAIT 1

#define RX_IRQ_MASK IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
#define TX_IRQ_MASK IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT

#define RX_IDENTITY_MASK 0xFF
#define BATTERY_VOLTAGE_MASK 0xFF00

extern SX128XLT LT;

extern unsigned long frequency;

TaskResult sendThrottlePacket(unsigned long now);