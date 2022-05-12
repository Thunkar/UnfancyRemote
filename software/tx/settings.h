//#define DUAL_THROTTLE

#ifdef DUAL_THROTTLE
const float BRAKE_SENSITIVITY = 5;
#endif

// Pairing

const unsigned int channel = 40;
const unsigned int TXIdentity = 2; 

// Battery levels

const unsigned long VREF = 1148566;

const float REMOTE_BATTERY_CELL_V_THR[] = { 4.0, 3.8, 3.7, 3.5, 3.3 };

const float BOARD_BATTERY_CELL_V_THR[] = { 4.0, 3.8, 3.7, 3.5, 3.3 };
const unsigned int BOARD_CELL_S = 12;
