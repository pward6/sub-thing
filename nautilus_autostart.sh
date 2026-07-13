#!/usr/bin/env bash
# ============================================================
# nautilus_autostart.sh - headless autonomous mission launcher (NO tmux).
#
# Run directly by nautilus-mission.service on boot. Brings up the stack,
# WAITS until the topics qualify.py needs actually exist (fixing the boot
# race that produced 97-byte early-death logs), then runs the hard-code
# mission in the foreground. All output goes to journald:
#     journalctl -u nautilus-mission -f
#
# The hardware kill switch is the interlock. Tune the mission params below.
# ============================================================
set -o pipefail

export HOME=/home/nano
source /opt/ros/humble/setup.bash
[ -f "$HOME/nautilus_ws/install/setup.bash" ] && source "$HOME/nautilus_ws/install/setup.bash"
cd "$HOME/nautilus_ws" || exit 1

# Kill the WHOLE stack, not just nautilus_up.sh: it backgrounds router/MAVROS/
# Nucleus, so killing only its PID orphans them and they keep holding the FCU
# serial + UDP ports 14540/14555 -> next run fails "port still in use".
kill_stack() {
  pkill -9 -f mavros_node   2>/dev/null
  pkill -9 -f nucleus_node  2>/dev/null
  pkill -9 mavlink-routerd  2>/dev/null
  pkill -9 -f qualify.py    2>/dev/null
}
# Always clean up on exit (timeout, mission end, or systemd stop).
trap kill_stack EXIT

echo "[autostart] clearing any leftover stack/mission processes"
kill_stack
sleep 3   # let the UDP ports 14540/14555 actually release before preflight

# FastRTPS (the default RMW) keeps shared-memory segments in /dev/shm. Stale
# ones left by repeated restarts break SERVICE discovery (topics still work),
# which shows up as "arm: service unavailable" even though the stack is up.
echo "[autostart] clearing stale FastRTPS shared-memory segments"
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/*fastdds* 2>/dev/null
sleep 1

echo "[autostart] bringing up stack (nautilus_up.sh)"
./nautilus_up.sh &
STACK_PID=$!

echo "[autostart] waiting for INS + MAVROS state topics (up to 180s)"
END=$(( $(date +%s) + 180 ))
until ros2 topic list 2>/dev/null | grep -q "/nucleus_node/ins_packets" \
   && ros2 topic list 2>/dev/null | grep -q "/mavros/state"; do
  if [ "$(date +%s)" -gt "$END" ]; then
    echo "[autostart] stack not ready within 180s -- aborting, NOT arming."
    exit 1   # trap kill_stack runs
  fi
  sleep 2
done
echo "[autostart] topics seen (daemon cache). Settling 10s."
sleep 10

# ros2 topic list reads the ROS daemon's CACHED graph, so it can report topics
# whose publisher already died. Verify the actual processes are alive before
# launching qualify.py -- otherwise we'd arm against a dead stack.
if ! pgrep -f mavros_node >/dev/null || ! pgrep -f nucleus_node >/dev/null; then
  echo "[autostart] STACK DIED after coming up (mavros/nucleus not running). NOT arming."
  echo "[autostart]   mavros pids: [$(pgrep -f mavros_node | tr '\n' ' ')]  nucleus pids: [$(pgrep -f nucleus_node | tr '\n' ' ')]  router: [$(pgrep mavlink-routerd | tr '\n' ' ')]"
  echo "[autostart]   -> the stack is not staying up. See the [stack] lines above for why (MAVROS connect? Nucleus 'too many connections'?)."
  exit 1
fi
echo "[autostart] stack processes alive. Launching hard-code mission."

# ---- MISSION PARAMS (tune here) ----
python3 scripts/qualify.py --ros-args \
  -p hard_code_enable:=true \
  -p hard_code_open_loop:=true \
  -p hard_code_descend_seconds:=4.0 \
  -p hard_code_descend_thrust:=0.4 \
  -p hard_code_forward_seconds:=8.0 \
  -p arm_delay:=30.0
# add  -p altitude_sign:=-1.0  above if it needs the inverted down direction

echo "[autostart] mission finished. Tearing down stack."
# trap kill_stack runs on exit
