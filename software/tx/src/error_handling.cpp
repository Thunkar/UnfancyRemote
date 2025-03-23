#include "error_handling.h"

void clearError() {
  state.error = false;
}

void setError(char reason[]) {
  state.error = true;
  stats.errors++;
  strcpy(stats.errorReason, reason);
}