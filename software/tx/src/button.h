#pragma once

#include "Arduino.h"
#include "board.h"
#include "scheduler.h"
#include "mode.h"

#define DEBOUNCE_DELAY_US 100 * 1e3
#define OFF_DELAY_US 1500 * 1e3    
#define CHANGE_MODE_DELAY_US 500 * 1e3

TaskResult checkButton(unsigned long now);