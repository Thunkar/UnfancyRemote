#pragma once

#include <Arduino.h>
#include "board.h"
#include "settings.h"
#include "state.h"
#include "utils.h"

TaskResult checkBattery(unsigned long now);