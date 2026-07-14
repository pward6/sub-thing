#!/usr/bin/env bash
# ============================================================
# nautilus_autostart.sh - headless autonomous mission launcher (NO tmux).
#
# Run directly by nautilus-mission.service on boot. Brings up router + MAVROS
# (NO Nucleus -- see SKIP_NUCLEUS), waits for MAVROS to be up, then runs the
# open-loop hard-code mission with no_nucleus (no heading hold). Output ->
#     journalctl -u nautilus-mission -f
#
# The hardware kill switch is the interlock. Tune the mission params below.
# ============================================================
set -o pipefail

export HOME=/home/nano
source /opt/ros/humble/setup.bash
[ -f "$HOME/nautilus_ws/install/setup.bash" ] && source "$HOME/nautilus_ws/install/setup.bash"
cd "$HOME/nautilus_ws" || exit 1

# Router + MAVROS only. The Nucleus DVL wouldn't connect and dragged the whole
# stack down; the open-loop mission doesn't need it (loses heading hold only).
export SKIP_NUCLEUS=1

# Kill the WHOLE stack, not just nautilus_up.sh: it backgrounds router/MAVROS,
# so killing only its PID orphans them and they keep holding the FCU serial +
# UDP ports 14540/14555 -> next run fails "port still in use".
kill_stack() {
  # Kill nautilus_up.sh FIRST (uncatchable SIGKILL) so it can't run its own
  # hanging cleanup trap; then its children.
  pkill -9 -f nautilus_up.sh 2>/dev/null
  pkill -9 -f mavros_node    2>/dev/null
  pkill -9 -f nucleus_node   2>/dev/null
  pkill -9 mavlink-routerd   2>/dev/null
  pkill -9 -f qualify.py     2>/dev/null
  # The ros2 CLI daemon (spawned by our topic-list polling) otherwise lingers
  # in the service cgroup and makes systemd wait the full stop-timeout, marking
  # the run Failed even though the mission completed.
  pkill -9 -f ros2cli        2>/dev/null
}
trap kill_stack EXIT

echo "[autostart] clearing any leftover stack/mission processes"
kill_stack
sleep 3   # let UDP ports 14540/14555 release before preflight

echo "[autostart] clearing stale FastRTPS shared-memory segments"
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/*fastdds* 2>/dev/null
sleep 1

# Bring up the stack and wait for the Cube to actually CONNECT (connected:true),
# RETRYING the whole bring-up if it doesn't. On a cold power-up the Cube
# (ArduSub) is still booting and may not heartbeat within one attempt's window;
# a fresh bring-up on the next attempt catches it. This is what makes cold-boot
# deployment reliable instead of finicky.
CONNECTED=false
for attempt in 1 2 3 4 5; do
  echo "[autostart] stack bring-up attempt $attempt (router + MAVROS, no Nucleus)"
  ./nautilus_up.sh &
  STACK_PID=$!
  END=$(( $(date +%s) + 70 ))
  while [ "$(date +%s)" -lt "$END" ]; do
    if pgrep -f mavros_node >/dev/null \
       && timeout 4 ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; then
      CONNECTED=true
      break
    fi
    sleep 3
  done
  [ "$CONNECTED" = true ] && break
  echo "[autostart] Cube not connected on attempt $attempt (still booting / link down?). Restarting stack."
  kill_stack
  sleep 4
done
if [ "$CONNECTED" != true ]; then
  echo "[autostart] Cube never sent connected:true after $attempt attempts. NOT arming."
  echo "[autostart]   -> check Cube power and the CP2102<->TELEM serial lead on the vehicle."
  exit 1
fi
echo "[autostart] Cube CONNECTED (connected:true). Settling 5s, then launching mission."
sleep 5

# ---- MISSION PARAMS (tune here) ----
python3 scripts/qualify.py --ros-args \
  -p hard_code_enable:=true \
  -p hard_code_open_loop:=true \
  -p no_nucleus:=true \
  -p hard_code_descend_seconds:=2.0 \
  -p hard_code_descend_thrust:=0.4 \
  -p hard_code_forward_seconds:=4.0 \
  -p hard_code_forward_downthrust:=0.0 \
  -p altitude_sign:=1.0 \
  -p arm_delay:=5.0

echo "[autostart] mission finished. Tearing down stack."
# trap kill_stack runs on exit
