#pragma once

#include <Arduino.h>
#include "state.h"
#include "stats.h"

#define MAX_LOOP_TIME_US 5000 

typedef TaskResult (*task)(unsigned long);
void schedule(task tasks[]);