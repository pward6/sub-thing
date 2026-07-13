#!/usr/bin/env bash
# ============================================================
# hard_code_run.sh - one-command dead-reckoned qualify.
#
#   Descend HARD_CODE_DOWN_FEET below the launch altitude, then drive
#   HARD_CODE_FORWARD_FEET forward on the heading captured at arm, then
#   disarm. No vision, no gate, no camera -- this is the minimum needed to
#   qualify (get down, get through). It drives qualify.py's hard-code mode;
#   see the module docstring in qualify.py for the details and cautions.
#
# EDIT THE THREE TOGGLES BELOW, then on the Jetson just run:
#
#     ./hard_code_run.sh
#
# Bench test above water first (no MAVROS/Nucleus/camera needed -- fabricates
# every sensor and refuses to arm or thrust):
#
#     ./hard_code_run.sh -p sim_sensors:=true
#
# Any extra args after the script name are passed straight through to
# qualify.py, so you can also override the toggles without editing this file:
#
#     ./hard_code_run.sh -p hard_code_down_distance:=4 -p hard_code_forward_distance:=8
# ============================================================
# NOTE: no 'set -u' here. ROS setup.bash references unbound variables
# (AMENT_TRACE_SETUP_FILES etc.), and nounset makes sourcing it abort.
set -o pipefail

# ---- THE THREE TOGGLES ----
HARD_CODE_ENABLE=true      # false -> normal (vision) qualify run
HARD_CODE_DOWN_FEET=3.0    # ft to descend below the launch altitude
HARD_CODE_FORWARD_FEET=10.0  # ft to drive forward on the captured heading
# ---------------------------

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Best-effort ROS sourcing so this works in a fresh shell / new SSH session.
# Adjust the workspace path if yours differs.
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$HOME/nautilus_ws/install/setup.bash" ] && source "$HOME/nautilus_ws/install/setup.bash"

echo "hard_code_run.sh: enable=$HARD_CODE_ENABLE down=${HARD_CODE_DOWN_FEET}ft forward=${HARD_CODE_FORWARD_FEET}ft"
echo "extra args: $*"

exec python3 "$DIR/qualify.py" --ros-args \
  -p hard_code_enable:="$HARD_CODE_ENABLE" \
  -p hard_code_down_distance:="$HARD_CODE_DOWN_FEET" \
  -p hard_code_forward_distance:="$HARD_CODE_FORWARD_FEET" \
  "$@"
