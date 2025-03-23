#pragma once

#include <SX128XLT.h>
#include <ProgramLT_Definitions.h>
#include "board.h"
#include "utils.h"
#include "state.h"
#include "config.h"
#include "error_handling.h"	
#include "stats.h"

extern SX128XLT LT;

bool receiveThrottlePacket(unsigned long now);