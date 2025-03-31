#include "scheduler.h"

// Scheduler

const unsigned long periods[N_TASKS] = { 10, 10, 200, 1000, 100, 50, 20, 50, 2000 };

void schedule(task tasks[N_TASKS]) {
  for(int i = 0; i < N_TASKS; i++) {
    unsigned long now = micros();
    if(state.activeTasks[i] && (now >= state.nextRun[i])) {
      TaskResult result = tasks[i](now);
      if(result.success) {
        stats.successes[i]++;
      } else {
        stats.failures[i]++;
      }
      state.nextRun[i] = now + periods[i]*1e3 + result.offset;
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