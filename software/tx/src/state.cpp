#include "state.h"

State state = { 
    // Throttle value
    ENCODED_HALF, 
    ENCODED_HALF,
    ENCODED_HALF,
    // Button state
    LOW,
    // Modes 
    0, 
    0, 
    0, 
    true, 
    false,
    // Battery voltages
    -1.0, 
    0.0, 
    0.0, 
    // RF
    false,
    // Next time tasks should be run in us
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    // Active tasks (server disabled by default)
    { 1, 1, 1, 1, 1, 1, 1, 0, 1 },
    // Error
    false
};