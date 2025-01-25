#include "error_handling.h"

void clearError() {
  state.error = false;
}

void setError(char reason[]) {
  state.errors++;
  state.error = true;
  strcpy(state.errorReason, reason);
}