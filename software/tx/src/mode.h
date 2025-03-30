#pragma once

#include "board.h"
#include "scheduler.h"
#include "LED.h"
#include "motor.h"
#include "state.h"

#define TRANSITION_DELAY_US 250 * 1e3

void changeMode(int mode);

TaskResult displayMode(unsigned long now);