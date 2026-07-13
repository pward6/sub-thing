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

echo "[autostart] clearing any leftover stack/mission processes"
pkill -9 -f mavros_node   2>/dev/null
pkill -9 -f nucleus_node  2>/dev/null
pkill -9 mavlink-routerd  2>/dev/null
pkill -9 -f qualify.py    2>/dev/null
sleep 2

echo "[autostart] bringing up stack (nautilus_up.sh)"
./nautilus_up.sh &
STACK_PID=$!

echo "[autostart] waiting for INS + MAVROS state topics (up to 180s)"
END=$(( $(date +%s) + 180 ))
until ros2 topic list 2>/dev/null | grep -q "/nucleus_node/ins_packets" \
   && ros2 topic list 2>/dev/null | grep -q "/mavros/state"; do
  if [ "$(date +%s)" -gt "$END" ]; then
    echo "[autostart] stack not ready within 180s -- aborting, NOT arming."
    kill "$STACK_PID" 2>/dev/null
    exit 1
  fi
  sleep 2
done
echo "[autostart] stack up. Settling 10s for services to advertise."
sleep 10
echo "[autostart] launching hard-code mission."

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
kill "$STACK_PID" 2>/dev/null
