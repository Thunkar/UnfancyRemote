#include "stats.h"

const char *TASK_NAMES[] = { "sendThrottlePacket", "readThrottle", "checkButton", "checkBattery", "displayMode", "setLEDs", "setMotor", "doServerWork", "printStats" };

unsigned long lastRun = 0;

Stats stats = {
    // Successes
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    // Failures
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    // Times
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 10000000, 10000000, 10000000, 10000000, 10000000, 10000000, 10000000, 10000000, 10000000 },
    // Loops
    0,
    // Packets
    0,
    0,
    // RFWaits
    0,
    0,
    0,
    0,
    // Errors
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
    stats.timeWaitingForRX = 0;
    stats.timeWaitingForTX = 0;
    stats.TXWaits = 0;
    stats.RXWaits = 0;
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

TaskResult printStats(unsigned long now) {
  #ifdef DEBUG
  Serial.print("Connected: ");
  Serial.println(state.isConnected);
  float ellapsed = (now - lastRun)/1e6;
  Serial.print(F("Ellapsed: "));
  Serial.print(ellapsed);
  Serial.print(F("s | VBat: "));
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
    long executions = stats.successes[i] + stats.failures[i];
    float frequency = stats.successes[i] / ellapsed;
    float mean = stats.times[i] / (float)executions;
    sprintf(prBuffer, "%-20s | %6.2fHz | %5dus | ~%7.2fus | %5dus | %.2f", TASK_NAMES[i], frequency, stats.minTimes[i], mean, stats.maxTimes[i], stats.successes[i]/(float)executions);
    Serial.print(prBuffer);
    Serial.println("");
  }
  Serial.println(F("------------------------------------------------------------------------"));
  float loopFrequency = stats.loops / ellapsed;
  Serial.print(F("Loop frequency: "));
  Serial.print(loopFrequency);
  Serial.println(F("Hz"));
  int packetsPerSecond = round(stats.packets / ellapsed);
  Serial.print(F("Packets/s: "));
  Serial.println(packetsPerSecond);
  int TMPacketsPerSecond = round(stats.TMPackets / ellapsed);
  Serial.print(F("TM packets/s: "));
  Serial.println(TMPacketsPerSecond);
  Serial.println(F("RF waits: "));
  float RXWaitMeanUs = stats.timeWaitingForRX / stats.RXWaits;
  char meanTimeWaitingRXBuffer[50];
  sprintf(meanTimeWaitingRXBuffer, "%-40s %.2fus", "- Mean time waiting for RX:", RXWaitMeanUs); 
  Serial.print(meanTimeWaitingRXBuffer);
  Serial.println("");
  float RXWaitsPerSecond = stats.RXWaits / ellapsed;
  char RXWaitsPerSecondBuffer[50];
  sprintf(RXWaitsPerSecondBuffer, "%-40s %.2f", "- RX waits/s: ", RXWaitsPerSecond);
  Serial.print(RXWaitsPerSecondBuffer);
  Serial.println("");
  float TXWaitMeanUs = stats.timeWaitingForTX / stats.TXWaits;
  char meanTimeWaitingTXBuffer[50];
  sprintf(meanTimeWaitingTXBuffer, "%-40s %.2fus", "- Mean time waiting for TX:", TXWaitMeanUs); 
  Serial.print(meanTimeWaitingTXBuffer);
  Serial.println("");
  float TXWaitsPerSecond = stats.TXWaits / ellapsed;
  char TXWaitsPerSecondBuffer[50];
  sprintf(TXWaitsPerSecondBuffer, "%-40s %.2f", "- TX waits/s: ", TXWaitsPerSecond);
  Serial.println(TXWaitsPerSecondBuffer);
  Serial.println("");
  sprintf(titleBuffer, "%-23s | %8s", "Error code", "Count/s");
  Serial.println(titleBuffer);
  Serial.println(F("-------------------------------------"));
  for(int i = 0; i < ERROR_TYPES; i++) {
    char prBuffer[150];
    sprintf(prBuffer, "%-23s | %5.2f", getReason((ERROR_CODE)i), stats.errors[i] / ellapsed);
    Serial.println(prBuffer);
  }
  Serial.println(F("-------------------------------------"));
  #endif
  resetStats();
  lastRun = now;
  return { true, 0 };
}