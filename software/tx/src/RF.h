#pragma once

#include <SX128XLT.h>
#include <ProgramLT_Definitions.h>
#include "board.h"
#include "state.h"
#include "settings.h"
#include "config.h"
#include "error_handling.h"

extern SX128XLT LT;

void IRAM_ATTR processRFInterrupt();

extern unsigned long frequency;

bool receiveTMPacket(unsigned long now);
bool sendThrottlePacket(unsigned long now);
bool checkRXIRQError();
bool checkTXRXDone();