#pragma once

#include <Arduino.h>
#include "board.h"
#include "scheduler.h"
#include "state.h"
#include "mode.h"
#include "utils.h"

TaskResult checkBattery(unsigned long now);