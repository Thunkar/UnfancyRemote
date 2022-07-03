// Pairing: Choose a channel (0-40) and a transmitter identity (0-254) to prevent collisions with nearby remotes or other LoRa devices. These numbers must match the transmitter's.

const unsigned int channel = -1;
const unsigned int RXIdentity = -1; 

// Battery levels: VREF is the actual internal reference of the Arduino, that can differ from one board to another. It can be adjusted here, increasing this number if the battery is reading high or decreasing it if it's reading low. It is strongly encouraged to use a multimeter to verify the actual values.

const unsigned long VREF = 1148566;
