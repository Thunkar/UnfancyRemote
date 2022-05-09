// Battery levels

//#define DUAL_THROTTLE

#ifdef DUAL_THROTTLE
const float BRAKE_SENSITIVITY = 5;
#endif

const float REMOTE_BATTERY_CELL_V_THR[] = { 4.0, 3.8, 3.7, 3.5, 3.3 };

const float BOARD_BATTERY_CELL_V_THR[] = { 4.0, 3.8, 3.7, 3.5, 3.3 };
const unsigned int BOARD_CELL_S = 12;
