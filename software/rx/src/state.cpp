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
    // Last time tasks run in us
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    // Active tasks (server disabled by default)
    { 1, 1, 1, 1, 1, 1, 0, 1 },
};