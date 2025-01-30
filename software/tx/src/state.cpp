#include "state.h"

State state = { 
    // Throttle value
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
    true,
    -100, 
    -100,
    0,
    0,
    1,
    // Errors
    false, 
    0, 
    ""
};