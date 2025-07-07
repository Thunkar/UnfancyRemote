#include "scheduler.h"

// Scheduler

const unsigned long periods[N_TASKS] = { 10000, 0, 0, 0, 10000, 200000, 1000000, 100000, 50000, 20000, 50000, 2000000 };

void schedule(task tasks[N_TASKS]) {
  unsigned long start = micros();
  for(int i = 0; i < N_TASKS; i++) {
    unsigned long now = micros();
    if(state.activeTasks[i] && ((now - start) < MAX_LOOP_TIME_US) && ((now - state.lastRun[i]) >= periods[i])) {
      TaskResult result = tasks[i](now);
      if(result.success) {
        stats.successes[i]++;
      } else {
        stats.failures[i]++;
      }
      state.lastRun[i] = now;
      unsigned long end = micros();
      unsigned long ellapsed = end - now;
      stats.times[i]+=ellapsed;
      if(stats.maxTimes[i] < ellapsed) {
        stats.maxTimes[i] = ellapsed;
      } 
      if (stats.minTimes[i] > ellapsed) {
        stats.minTimes[i] = ellapsed;
      }
    }
  }
  stats.loops++;
}