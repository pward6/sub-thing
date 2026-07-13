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
}
trap kill_stack EXIT

echo "[autostart] clearing any leftover stack/mission processes"
kill_stack
sleep 3   # let UDP ports 14540/14555 release before preflight

echo "[autostart] clearing stale FastRTPS shared-memory segments"
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/*fastdds* 2>/dev/null
sleep 1

echo "[autostart] bringing up stack (router + MAVROS, no Nucleus)"
./nautilus_up.sh &
STACK_PID=$!

echo "[autostart] waiting for /mavros/state (up to 180s)"
END=$(( $(date +%s) + 180 ))
until ros2 topic list 2>/dev/null | grep -q "/mavros/state"; do
  if [ "$(date +%s)" -gt "$END" ]; then
    echo "[autostart] MAVROS not up within 180s -- aborting, NOT arming."
    exit 1   # trap kill_stack runs
  fi
  sleep 2
done
echo "[autostart] MAVROS topic seen. Settling 10s."
sleep 10

# ros2 topic list reads the daemon's CACHED graph and can report a dead
# publisher. Verify MAVROS is actually alive before arming.
if ! pgrep -f mavros_node >/dev/null; then
  echo "[autostart] MAVROS DIED after coming up. NOT arming."
  echo "[autostart]   mavros pids: [$(pgrep -f mavros_node | tr '\n' ' ')]  router: [$(pgrep mavlink-routerd | tr '\n' ' ')]"
  exit 1
fi
echo "[autostart] MAVROS alive. Launching open-loop hard-code mission."

# ---- MISSION PARAMS (tune here) ----
python3 scripts/qualify.py --ros-args \
  -p hard_code_enable:=true \
  -p hard_code_open_loop:=true \
  -p no_nucleus:=true \
  -p hard_code_descend_seconds:=4.0 \
  -p hard_code_descend_thrust:=0.4 \
  -p hard_code_forward_seconds:=8.0 \
  -p altitude_sign:=-1.0 \
  -p arm_delay:=30.0

echo "[autostart] mission finished. Tearing down stack."
# trap kill_stack runs on exit
