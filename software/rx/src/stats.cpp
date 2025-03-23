#include "stats.h"

char *TASK_NAMES[] = { "receiveThrottlePacket", "writePPMValue", "checkBattery", "printStats", "doServerWork" };

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
    // RFWaits
    0,
    0,
    0,
    0,
    // Errors
    0,
    ""
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
    stats.errors = 0;
    strcpy(stats.errorReason, "");
}

bool printStats(unsigned long now) {
  #ifdef DEBUG
  if(stats.errors > 0) {
    Serial.println(F("////////ERROR//////////"));
    Serial.println(stats.errorReason);
    Serial.println(F("//////////////////////"));
  }
  Serial.print(F("Frequency: "));
  Serial.print(config.frequency);
  Serial.println(F("Hz"));
  float ellapsed = (now - state.lastRun[4])/1000;
  Serial.print(F("Ellapsed: "));
  Serial.print(ellapsed);
  Serial.print(F("s | VBat: "));
  Serial.print(state.boardVoltage);
  Serial.print(F("V | SNR: "));
  Serial.print(state.currentSNR);
  Serial.print(F("dB | RSSI: "));
  Serial.print(state.currentRSSI);
  Serial.println(F("dBm"));
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
    long executions = stats.successes[i] + stats.failures[i];
    float frequency = stats.successes[i] / ellapsed;
    float mean = stats.times[i] / (float)executions;
    sprintf(prBuffer, "%-23s | %6.2fHz | %6dus | ~%8.2fus | %6dus | %.2f", TASK_NAMES[i], frequency, stats.minTimes[i], mean, stats.maxTimes[i], stats.successes[i]/(float)executions);
    Serial.print(prBuffer);
    Serial.println("");
  }
  Serial.println(F("-----------------------------------------------------------------------------"));
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
  Serial.print(TXWaitsPerSecondBuffer);
  Serial.println("");
  Serial.print(F("Errors: "));
  Serial.println(stats.errors);
  #endif
  resetStats();
  return true;
}