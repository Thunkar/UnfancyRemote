#include "stats.h"

char *TASK_NAMES[] = { "receiveThrottlePacket", "writePPMValue", "checkBattery", "doServerWork", "printStats" };

unsigned long lastRun = 0;

Stats stats = {
    // Successes
    { 0, 0, 0, 0, 0 },
    // Failures
    { 0, 0, 0, 0, 0 },
    // Times
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0 },
    { 10000000, 10000000, 10000000, 10000000, 10000000 },
    // Loops
    0,
    // Packets
    0,
    0,
    0,
    0,
    0,
    // RFWaits
    0,
    0,
    0,
    0,
    // RxOffsets
    0,
    // RSSI, SNR
    -10000000,
    10000000,
    0,
    0,
    // Errors
    { 0, 0, 0, 0, 0 }
};

ComputedStats computedStats = {
    // Mean packet time
    0,
    // TMPackets per second
    0,
    // Packets per second
    0,
    // Task frequencies
    { 0, 0, 0, 0, 0 },
    // Task mean times
    { 0, 0, 0, 0, 0 },
    // Task ratios
    { 0, 0, 0, 0, 0 },
    // Loop frequency
    0,
    // RX wait mean
    0,
    // RX waits per second
    0,
    // TX wait mean
    0,
    // TX waits per second
    0,
    // Mean RX offsets
    0,
    // Errors per second
    { 0, 0, 0, 0, 0 }
};

void resetStats() {
    for(int i = 0; i < N_TASKS; i++) {
      stats.successes[i] = 0;
      stats.failures[i] = 0;
      stats.times[i] = 0;
      stats.maxTimes[i] = 0;
      stats.minTimes[i] = 10000000;
    }
    stats.loops = 0;
    stats.packets = 0;
    stats.packetTimes = 0;
    stats.maxPacketTime = 0;
    stats.minPacketTime = 10000000;
    stats.TMPackets = 0;
    stats.timeWaitingForRX = 0;
    stats.timeWaitingForTX = 0;
    stats.TXWaits = 0;
    stats.RXWaits = 0;
    stats.rxOffsets = 0;
    stats.RSSI = 0;
    stats.SNR = 0;
    stats.minSNR = 10000000;
    stats.maxSNR = -10000000;
    for(int i = 0; i < ERROR_TYPES; i++) {
        stats.errors[i] = 0;
    }
}

void setError(ERROR_CODE code) {
  stats.errors[code]++;
}

char* getReason(ERROR_CODE code) {
  switch(code) {
    case IRQ_ERROR:
      return "IRQ Error";
    case INCORRECT_IDENTITY:
      return "Incorrect identity";
    case TX_TIMEOUT:
      return "TX timeout";
    case RX_TIMEOUT:
      return "RX timeout";
    case DISCONNECTED:
      return "Disconnected";
    default:
      return "Unknown error";
  }
}

void computeStats(unsigned long now) {
  // In seconds
  float ellapsed = (now - lastRun)/1e6;
  for(int i = 0; i < N_TASKS; i++) {
    if(!state.activeTasks[i]) {
        continue;
    }
    long executions = stats.successes[i] + stats.failures[i];
    computedStats.taskFrequencies[i] = executions / ellapsed;
    computedStats.taskMeanTimes[i] = stats.times[i] / (float)executions;
    computedStats.taskRatios[i] = stats.successes[i]/(float)executions;
  }
  computedStats.loopFrequency = stats.loops / ellapsed;
  computedStats.packetsPerSecond = round(stats.packets / ellapsed);
  computedStats.TMPacketsPerSecond = round(stats.TMPackets / ellapsed);
  computedStats.meanPacketTime = stats.packetTimes / (float)stats.packets;

  computedStats.meanRXOffsets = stats.rxOffsets / (float)stats.successes[0];

  computedStats.RXWaitMean = stats.timeWaitingForRX / stats.RXWaits;
  computedStats.RXWaitsPerSecond = stats.RXWaits / ellapsed;
  computedStats.TXWaitMean = stats.timeWaitingForTX / stats.TXWaits;
  computedStats.TXWaitsPerSecond = stats.TXWaits / ellapsed;
  for(int i = 0; i < ERROR_TYPES; i++) {
    computedStats.errorsPerSecond[i] = stats.errors[i] / ellapsed;
  }
  lastRun = now;
}

TaskResult printStats(unsigned long now) {
  computeStats(now);
  #ifdef DEBUG
  Serial.print("Connected: ");
  Serial.println(state.isConnected);
  Serial.print(F("VBat: "));
  Serial.print(state.boardVoltage);
  Serial.print(F("V | SNR: "));
  Serial.print(stats.SNR);
  Serial.print(F("dB | RSSI: "));
  Serial.print(stats.RSSI);
  Serial.println(F("dBm"));
  Serial.print(F("Frequency: "));
  Serial.print(config.frequency);
  Serial.println(F("Hz"));
  Serial.println("");
  char titleBuffer[150];
  sprintf(titleBuffer, "%-23s | %8s | %8s | %11s | %8s | %3s", "Task", "Freq", "Min", "Mean", "Max", "Ratio");
  Serial.println(titleBuffer);
  Serial.println(F("-----------------------------------------------------------------------------"));
  for(int i = 0; i < N_TASKS; i++) {
    if(!state.activeTasks[i]) {
        continue;
    }
    char prBuffer[150];
    sprintf(prBuffer, "%-23s | %6.2fHz | %6dus | ~%8.2fus | %6dus | %.2f", TASK_NAMES[i], computedStats.taskFrequencies[i], stats.minTimes[i], computedStats.taskMeanTimes[i], stats.maxTimes[i], computedStats.taskRatios[i]);
    Serial.print(prBuffer);
    Serial.println("");
  }
  Serial.println(F("-----------------------------------------------------------------------------"));
  Serial.print(F("Loop frequency: "));
  Serial.print(computedStats.loopFrequency);
  Serial.println(F("Hz"));
  Serial.print(F("Packets/s: "));
  Serial.println(computedStats.packetsPerSecond);
  char packetTimesBuffer[50];
  Serial.print(F("Packet timings (min/mean/max): "));
  sprintf(packetTimesBuffer, "%5.2fms / ~%5.2fms / %5.2fms", stats.minPacketTime / 1000.0, computedStats.meanPacketTime / 1000.0, stats.maxPacketTime / 1000.0);
  Serial.println(packetTimesBuffer);
  Serial.print(F("TM packets/s: "));
  Serial.println(computedStats.TMPacketsPerSecond);
  Serial.println(F("Sync: "));
  char rxOffsetsBuffer[50];
  sprintf(rxOffsetsBuffer, "- Rx offsets: %6.2fus", computedStats.meanRXOffsets);
  Serial.println(rxOffsetsBuffer);
  Serial.println(F("RF waits: "));
  char meanTimeWaitingRXBuffer[50];
  sprintf(meanTimeWaitingRXBuffer, "%-40s %.2fus", "- Mean time waiting for RX:", computedStats.RXWaitMean); 
  Serial.print(meanTimeWaitingRXBuffer);
  Serial.println("");
  char RXWaitsPerSecondBuffer[50];
  sprintf(RXWaitsPerSecondBuffer, "%-40s %.2f", "- RX waits/s: ", computedStats.RXWaitsPerSecond);
  Serial.print(RXWaitsPerSecondBuffer);
  Serial.println("");
  char meanTimeWaitingTXBuffer[50];
  sprintf(meanTimeWaitingTXBuffer, "%-40s %.2fus", "- Mean time waiting for TX:", computedStats.TXWaitMean); 
  Serial.print(meanTimeWaitingTXBuffer);
  Serial.println("");
  char TXWaitsPerSecondBuffer[50];
  sprintf(TXWaitsPerSecondBuffer, "%-40s %.2f", "- TX waits/s: ", computedStats.TXWaitsPerSecond);
  Serial.println(TXWaitsPerSecondBuffer);
  Serial.println("");
  sprintf(titleBuffer, "%-23s | %8s", "Error code", "Count/s");
  Serial.println(titleBuffer);
  Serial.println(F("-------------------------------------"));
  for(int i = 0; i < ERROR_TYPES; i++) {
    char prBuffer[150];
    sprintf(prBuffer, "%-23s | %5.2f", getReason((ERROR_CODE)i), computedStats.errorsPerSecond[i]);
    Serial.println(prBuffer);
  }
  Serial.println(F("-------------------------------------"));
  #endif
  resetStats();
  return { true, 0 };
}