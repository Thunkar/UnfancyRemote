#include "button.h"

unsigned long lastPressedTime = 0;

TaskResult checkButton(unsigned long now) {
  int buttonState;
  int reading = digitalRead(BUTTON);
  if (reading != state.lastButtonState) {
    lastPressedTime = now;
  }

  if(now - lastPressedTime > DEBOUNCE_DELAY_US) {
      buttonState = reading;
  }

  if (now - lastPressedTime > OFF_DELAY_US) {
    if (buttonState) {
      digitalWrite(MOTOR, HIGH);
      digitalWrite(ON, LOW);
      digitalWrite(4, LOW);
      pinMode(BUTTON, INPUT_PULLDOWN);
      delay(100000000000);
    }
  }

  if(now - lastPressedTime > CHANGE_MODE_DELAY_US) {
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
  return { true };
}
