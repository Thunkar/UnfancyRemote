#pragma once

#include <Arduino.h>
#include "board.h"
#include "scheduler.h"
#include "state.h"
#include "config.h"
#include "button.h"
#include "utils.h"

#define BRAKE_SENSITIVITY 20

TaskResult readThrottle(unsigned long now);