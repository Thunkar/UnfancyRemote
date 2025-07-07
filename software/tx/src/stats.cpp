#include "stats.h"

const char *TASK_NAMES[] = { "sendThrottlePacket", "checkRFStatus", "receiveTMPacket", "handleTMPacket", "readThrottle", "checkButton", "checkBattery", "displayMode", "setLEDs", "setMotor", "doServerWork", "printStats" };

unsigned long lastRun = 0;

Stats stats = {
    // Successes
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    // Failures
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    // Times
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 10000000, 10000000, 10000000, 10000000, 10000000, 10000000, 10000000, 10000000, 10000000, 10000000, 10000000, 10000000 },
    // Loops
    0,
    // Packets
    0,
    0,
    // Errors
    { 0, 0, 0, 0, 0 }
};

ComputedStats computedStats = {
  // TMPackets per second
  0,
  // Packets per second
  0,
  // Task frequencies
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  // Task mean times
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  // Task ratios
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  // Loop frequency
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
    stats.TMPackets = 0;
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
    computedStats.taskFrequencies[i] = stats.successes[i] / ellapsed;
    computedStats.taskMeanTimes[i] = stats.times[i] / (float)executions;
    computedStats.taskRatios[i] = stats.successes[i]/(float)executions;
  }
  computedStats.loopFrequency = stats.loops / ellapsed;
  computedStats.packetsPerSecond = round(stats.packets / ellapsed);
  computedStats.TMPacketsPerSecond = round(stats.TMPackets / ellapsed);
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
  Serial.print(state.batteryVoltage);
  Serial.print(F("V | Mode: "));
  Serial.println(state.currentDisplayMode);
  Serial.print(F("Frequency: "));
  Serial.print(config.frequency);
  Serial.println(F("Hz"));
  Serial.print(F("Board V: "));
  Serial.print(state.boardVoltage);
  Serial.print(F(" ("));
  Serial.print(state.boardCellVoltage);
  Serial.println(F(")"));
  Serial.print(F("Calibration: "));
  Serial.print(config.calBrake);
  Serial.print(F(" | "));
  Serial.print(config.centerAcc);
  Serial.print(F(" | "));
  Serial.print(config.calAcc);
  Serial.print(F(" | Inverted: "));
  Serial.println(config.inverted ? "y" : "n");
  Serial.println("");
  char titleBuffer[150];
  sprintf(titleBuffer, "%-20s | %8s | %7s | %10s | %7s | %3s", "Task", "Freq", "Min", "Mean", "Max", "Ratio");
  Serial.println(titleBuffer);
  Serial.println(F("-----------------------------------------------------------------------"));
  for(int i = 0; i < N_TASKS; i++) {
    if(!state.activeTasks[i]) {
        continue;
    }
    char prBuffer[150];
    sprintf(prBuffer, "%-20s | %6.2fHz | %5dus | ~%7.2fus | %5dus | %.2f", TASK_NAMES[i], computedStats.taskFrequencies[i], stats.minTimes[i], computedStats.taskMeanTimes[i], stats.maxTimes[i], computedStats.taskRatios[i]);
    Serial.print(prBuffer);
    Serial.println("");
  }
  Serial.println(F("------------------------------------------------------------------------"));
  Serial.print(F("Loop frequency: "));
  Serial.print(computedStats.loopFrequency);
  Serial.println(F("Hz"));
  Serial.print(F("Packets/s: "));
  Serial.println(computedStats.packetsPerSecond);
  Serial.print(F("TM packets/s: "));
  Serial.println(computedStats.TMPacketsPerSecond);
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
  return { true };
}