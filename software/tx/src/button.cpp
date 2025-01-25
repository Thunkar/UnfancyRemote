#include "button.h"

unsigned long lastPressedTime = 0;
const unsigned long debounceDelay = 100;
const unsigned long offDelay = 1500;    
const unsigned long changeModeDelay = 500;

bool checkButton(unsigned long now) {
  int buttonState;
  int reading = digitalRead(BUTTON);
  if (reading != state.lastButtonState) {
    lastPressedTime = now;
  }

  if(now - lastPressedTime > debounceDelay) {
      buttonState = reading;
  }

  if (now - lastPressedTime > offDelay) {
    if (buttonState) {
      digitalWrite(MOTOR, HIGH);
      digitalWrite(ON, LOW);
      digitalWrite(4, LOW);
      pinMode(BUTTON, INPUT_PULLDOWN);
      delay(100000000000);
    }
  }

  if(now - lastPressedTime > changeModeDelay) {
    if(buttonState && state.currentDisplayMode != -1 && state.canChangeMode) {
      state.lastModeTransition = now;
      state.canChangeMode = false;
      changeMode(!state.currentDisplayMode);
    }
  }
  
  if(!reading && state.lastButtonState) {
    state.canChangeMode = true;
  }
  state.lastButtonState = reading;
  return true;
}
