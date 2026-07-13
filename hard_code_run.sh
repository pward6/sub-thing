#!/usr/bin/env bash
# ============================================================
# hard_code_run.sh - one-command dead-reckoned qualify.
#
# Defaults to OPEN-LOOP timed mode: no vision, and NO DVL/altimeter/INS
# position -- just thrust DOWN for a set time, let ALT_HOLD (the Cube's
# barometer, independent of the DVL) hold depth, then thrust FORWARD for a
# set time on the captured compass heading, then disarm. This is the robust
# minimum for qualifying when the DVL is unreliable: dip below a near-surface
# gate, hold depth, drive through. Amounts are SECONDS, not distance -- tune
# them by watching the pool. See qualify.py's module docstring for detail.
#
# EDIT THE TOGGLES BELOW, then on the Jetson (with MAVROS running) run:
#
#     ./hard_code_run.sh
#
# Bench test above water first, no hardware, refuses to arm or thrust:
#
#     ./hard_code_run.sh -p sim_sensors:=true
#
# Bench HARDWARE-IN-THE-LOOP (thrusters WILL spin, MAVROS must be up, props
# clear):
#
#     ./hard_code_run.sh -p sim_sensors:=true -p sim_thrust:=true
#
# If "down" makes it RISE, the z direction is backwards -- flip the sign:
#
#     ./hard_code_run.sh -p altitude_sign:=-1.0
#
# Any extra args after the script name pass straight through to qualify.py.
# ============================================================
# NOTE: no 'set -u' here. ROS setup.bash references unbound variables
# (AMENT_TRACE_SETUP_FILES etc.), and nounset makes sourcing it abort.
set -o pipefail

# ---- THE TOGGLES (open-loop timed) ----
HARD_CODE_ENABLE=true       # false -> normal (vision) qualify run
OPEN_LOOP=true              # true -> ignore DVL/altimeter/INS pos, use timed thrust
DESCEND_SECONDS=7.0         # s to thrust DOWN (keep short; ALT_HOLD then holds depth)
DESCEND_THRUST=0.4          # down-thrust fraction of full authority (0-1)
FORWARD_SECONDS=8.0         # s to thrust FORWARD through the gate
ALTITUDE_SIGN=-1.0          # down direction. Verified on this vehicle: -1.0 = down, +1.0 = up
# ---------------------------------------

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Best-effort ROS sourcing so this works in a fresh shell / new SSH session.
# Adjust the workspace path if yours differs.
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$HOME/nautilus_ws/install/setup.bash" ] && source "$HOME/nautilus_ws/install/setup.bash"

echo "hard_code_run.sh: enable=$HARD_CODE_ENABLE open_loop=$OPEN_LOOP down=${DESCEND_SECONDS}s@${DESCEND_THRUST} forward=${FORWARD_SECONDS}s sign=$ALTITUDE_SIGN"
echo "extra args: $*"

exec python3 "$DIR/qualify.py" --ros-args \
  -p hard_code_enable:="$HARD_CODE_ENABLE" \
  -p hard_code_open_loop:="$OPEN_LOOP" \
  -p hard_code_descend_seconds:="$DESCEND_SECONDS" \
  -p hard_code_descend_thrust:="$DESCEND_THRUST" \
  -p hard_code_forward_seconds:="$FORWARD_SECONDS" \
  -p altitude_sign:="$ALTITUDE_SIGN" \
  "$@"
