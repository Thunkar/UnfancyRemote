#include "state.h"

State state = { 
    // Throttle value
    ENCODED_HALF, 
    // Button state
    LOW,
    // Modes 
    false,
    // Battery voltage
    -1.0, 
    // RF
    true,
    -100, 
    -100,
    // Last time tasks were run in ms
    { 0, 0, 0, 0, 0 },
    // Active tasks (server disabled by default)
    { 1, 1, 1, 0, 1 },
    // Error
    false
};