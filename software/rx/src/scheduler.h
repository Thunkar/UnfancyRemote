#pragma once

#include <Arduino.h>
#include "state.h"
#include "stats.h"

typedef TaskResult (*task)(unsigned long);
void schedule(task tasks[]);