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
    0,
    0,
    0,
    // Errors
    false, 
    0, 
    ""
};