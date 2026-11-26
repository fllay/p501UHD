#!/bin/bash
# pin_gnb.sh: run-time thread placement for the gNB on pi5gnuc (i5-1340P: P-cores 0-7 as HT pairs, E-cores 8-15).
# Run after the gNB has started. lower_phy_tx/rx/ul and the p501 driver's USB event thread (SCHED_FIFO 60, named "gnb")
# each get their own physical P-core (siblings 1,3,5,7 left idle); every other gNB thread goes to the E-cores.
# The xHCI interrupts go to CPU 7, next to the USB event thread.
P=$(pgrep -x gnb) || { echo "no gnb"; exit 1; }
for T in /proc/$P/task/*; do
  tid=${T##*/}; n=$(cat $T/comm); pol=$(chrt -p $tid 2>/dev/null | awk -F: '/policy/{gsub(/ /,"",$2); print $2}'); pr=$(chrt -p $tid 2>/dev/null | awk -F: '/priority/{gsub(/ /,"",$2); print $2}')
  case "$n" in
    lower_phy_tx*) c=0 ;; lower_phy_rx*) c=2 ;; lower_phy_ul*) c=4 ;;
    gnb) [ "$pol" = "SCHED_FIFO" ] && [ "$pr" = "60" ] && c=6 || c=8-15 ;;
    *) c=8-15 ;;
  esac
  taskset -p -c $c $tid >/dev/null && printf "%-16s %6s -> %s\n" "$n" "$tid" "$c"
done
for irq in $(grep -i xhci /proc/interrupts | awk -F: '{print $1}'); do echo pi | sudo -S sh -c "echo 7 > /proc/irq/$irq/smp_affinity_list" 2>/dev/null; done
