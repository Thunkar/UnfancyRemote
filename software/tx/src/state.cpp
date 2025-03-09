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
    true,
    0,
    0,
    0,
    // Errors
    false, 
    0, 
    ""
};