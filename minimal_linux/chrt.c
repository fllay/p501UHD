#define _GNU_SOURCE
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
/* minimal chrt: chrt -f|-r|-o|-b|-i [-p] PRIO PID  (also: chrt -f PRIO cmd args) */
int main(int c, char **v) {
  int pol = SCHED_FIFO, i = 1, usepid = 0;
  for (; i < c && v[i][0] == '-' && v[i][1]; i++) {
    switch (v[i][1]) { case 'f': pol = SCHED_FIFO; break; case 'r': pol = SCHED_RR; break;
      case 'o': pol = SCHED_OTHER; break; case 'b': pol = SCHED_BATCH; break; case 'i': pol = SCHED_IDLE; break;
      case 'p': usepid = 1; break; default: break; }
  }
  if (c - i < 2) { fprintf(stderr, "usage: chrt -f|-r|-o [-p] PRIO PID|CMD...\n"); return 1; }
  struct sched_param sp = { .sched_priority = atoi(v[i]) };
  if (usepid) { if (sched_setscheduler(atoi(v[i+1]), pol, &sp)) { perror("sched_setscheduler"); return 1; } return 0; }
  if (sched_setscheduler(0, pol, &sp)) { perror("sched_setscheduler"); return 1; }
  execvp(v[i+1], v + i + 1); perror("exec"); return 1;
}
