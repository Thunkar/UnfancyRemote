#pragma once

#include <SX128XLT.h>
#include <ProgramLT_Definitions.h>
#include "board.h"
#include "utils.h"
#include "state.h"
#include "config.h"
#include "error_handling.h"

extern SX128XLT LT;

extern unsigned long frequency;

void IRAM_ATTR processRFInterrupt();
bool receiveThrottlePacket(unsigned long now);