#pragma once

#include "board.h"
#include "settings.h"
#include "LED.h"
#include "motor.h"
#include "state.h"

void changeMode(int mode);

bool displayMode(unsigned long now);