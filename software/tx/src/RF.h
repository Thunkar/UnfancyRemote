#pragma once

#include <SX128XLT.h>
#include <ProgramLT_Definitions.h>
#include "board.h"
#include "state.h"
#include "settings.h"
#include "error_handling.h"

extern SX128XLT LT;

void IRAM_ATTR processRFInterrupt();

bool checkTXRXDone();
bool checkRXIRQError();
bool receiveTMPacket(unsigned long now);
bool sendThrottlePacket(unsigned long now);
