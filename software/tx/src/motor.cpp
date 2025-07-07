#include "motor.h"

int motorStatus = LOW;
unsigned long motorPeriod = -1;
unsigned long lastMotorToggled = 0;
int motorResetCounter = -1;

void pulseMotor(int times, unsigned long period) {
  motorPeriod = period;
  motorResetCounter = times;
}

TaskResult setMotor(unsigned long now) {
  int currentStatus = motorStatus;
  if(motorPeriod == -1) {
    motorStatus = LOW;  
  } else if (motorPeriod == 0) {
    motorStatus = HIGH;
  } else if (now - lastMotorToggled >= motorPeriod) {
    int isPulsing = motorResetCounter != 0;
    if(isPulsing) {
      motorStatus = !motorStatus;
      if(motorStatus && motorResetCounter > 0){
        motorResetCounter--;
      }
      lastMotorToggled = now;
    } else {
      motorResetCounter = -1;
      motorPeriod = -1;
    }
  }
  if(currentStatus != motorStatus) {
    digitalWrite(MOTOR, motorStatus);
  }
  return { true };
}