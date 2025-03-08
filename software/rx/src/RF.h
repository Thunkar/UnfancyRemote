#pragma once

#include <SX128XLT.h>
#include <ProgramLT_Definitions.h>
#include "board.h"
#include "state.h"
#include "config.h"
#include "error_handling.h"

extern SX128XLT LT;

void IRAM_ATTR processRFInterrupt();

extern unsigned long frequency;

bool receiveThrottlePacket(unsigned long now);
bool sendTMPacket(unsigned long now);
bool checkRXIRQError();
bool checkTXRXDone();