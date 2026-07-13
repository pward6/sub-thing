#!/usr/bin/env bash
# ============================================================
# nautilus_session.sh - Bring up the Nautilus stack in tmux.
#
#   window 0: stack     nautilus_up.sh (router, MAVROS, Nucleus)
#   window 1: vision    pipe_detector.py
#   window 2: bag       ros2 bag record
#   window 3: shell     empty, for you
#   window 4: mission   gate_pass.py, ONLY if NAUTILUS_AUTOSTART=1
#
# gate_pass.py does NOT wait to see the gate and has NO arm countdown -- it
# arms as soon as the FCU is connected and an INS heading is available, then
# holds that heading and drives forward for a fixed duration. One thing
# stands between a boot and a spinning propeller:
#   - window 4 only exists when NAUTILUS_AUTOSTART=1 is in the environment
#
# There is no water interlock and no depth logic. gate_pass.py sets ALT_HOLD
# and never commands vertical thrust, so ArduSub holds whatever depth the
# vehicle was launched at.
#
# Launched by hand (no env var) you get windows 0-3 and start the mission
# yourself from window 3. That is what you want on a bench.
#
# Attach over SSH:   tmux attach -t nautilus
# Detach:            Ctrl+B then D
# Switch window:     Ctrl+B then 0/1/2/3/4
# Scroll back:       Ctrl+B then [   (q to exit)
# ============================================================
set -uo pipefail

SESSION="nautilus"
WS="$HOME/nautilus_ws"
SCRIPTS="$WS/scripts"
BAGS="$HOME/bags"
SRC="source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"

TOPICS="/mavros/state /mavros/imu/data /mavros/battery \
/nautilus/detections /nucleus_node/ins_packets \
/nucleus_node/bottom_track_packets"

mkdir -p "$BAGS"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "session '$SESSION' already exists. tmux attach -t $SESSION"
  exit 0
fi

# Keep panes alive after a command exits so you can read the error.
tmux new-session -d -s "$SESSION" -n stack
tmux set-option -t "$SESSION" remain-on-exit on

tmux send-keys -t "$SESSION":stack \
  "$SRC && cd $WS && ./nautilus_up.sh" C-m

# nautilus_up.sh takes ~20-30s: router settle, MAVROS connect, Nucleus
# connect + start. The detector only needs the ROS graph, but give it room.
tmux new-window -t "$SESSION" -n vision
tmux send-keys -t "$SESSION":vision \
  "$SRC && sleep 30 && python3 $SCRIPTS/pipe_detector.py" C-m

tmux new-window -t "$SESSION" -n bag
tmux send-keys -t "$SESSION":bag \
  "$SRC && sleep 35 && ros2 bag record -o $BAGS/run_\$(date +%F_%H%M%S) $TOPICS" C-m

tmux new-window -t "$SESSION" -n shell
tmux send-keys -t "$SESSION":shell "$SRC" C-m
tmux send-keys -t "$SESSION":shell \
  "echo 'Bench:  python3 $SCRIPTS/gate_pass.py --ros-args -p dry_run:=true'" C-m
tmux send-keys -t "$SESSION":shell \
  "echo 'Live :  python3 $SCRIPTS/gate_pass.py --ros-args -p forward_thrust:=0.30'" C-m
tmux send-keys -t "$SESSION":shell \
  "echo 'Abort:  gate_pass.py has no /nautilus/cmd/abort listener -- Ctrl+C its window, or the kill switch'" C-m

# ---- window 4: autonomous mission ----
# Only when explicitly asked. The systemd unit sets NAUTILUS_AUTOSTART=1;
# running this script by hand does not.
if [ "${NAUTILUS_AUTOSTART:-0}" = "1" ]; then
  tmux new-window -t "$SESSION" -n mission
  tmux send-keys -t "$SESSION":mission \
    "$SRC && sleep 20 && python3 $SCRIPTS/gate_pass.py --ros-args \
      -p forward_thrust:=0.30 \
      2>&1 | tee -a $BAGS/mission_\$(date +%F_%H%M%S).log" C-m
  echo "AUTOSTART: mission node will arm as soon as FCU + INS heading are ready -- no gate sighting, no countdown."
fi

echo "started. attach with:  tmux attach -t $SESSION"
