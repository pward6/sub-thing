#!/usr/bin/env bash
# ============================================================
# nautilus_up.sh - Full Nautilus stack, one command.
#
#   1. mavlink-router   (Cube Orange+ / ArduSub over CP2102)
#   2. MAVROS           -> waits for connected:true
#   3. nucleus_node     -> connects + starts DVL/INS streaming
#
# Ctrl+C tears everything down cleanly.
#
# ---- HARD-WON CONFIG (2026-07-08). Do not "simplify" these. ----
# * /etc/mavlink-router/main.conf must contain:
#       [UartEndpoint alpha]
#       Device=/dev/ttyUSB0
#       Baud=115200
#
#       [UdpEndpoint mavros]
#       Mode=Server
#       Address=0.0.0.0
#       Port=14540
#
# * MAVROS fcu_url MUST use two distinct ports:
#       udp://127.0.0.1:14555@127.0.0.1:14540
#   (bind 14555 locally, send to router's server on 14540)
#   Using "udp://127.0.0.1:14540@" makes MAVROS try to BIND 14540,
#   which the router already owns -> "udp:bind: Address already in use"
#   -> half-open link, VER timeouts, connected:false.
#
# * Run mavlink-routerd with NO ARGS (config file only). Passing
#   /dev/ttyUSB0 on the CLI *and* in main.conf opens the UART twice
#   and corrupts MAVLink traffic.
#
# * HARNESS: CP2102 <-> Cube TELEM2 is THREE WIRES ONLY.
#       TELEM2 pin2 (TX) -> CP2102 RX
#       TELEM2 pin3 (RX) -> CP2102 TX
#       TELEM2 pin6 (GND)-> CP2102 GND
#   Do NOT connect CP2102 5V to TELEM2 pin1. Both are 5V OUTPUTS;
#   tying them together back-feeds USB VBUS and corrupts the UART.
#
# * Never run cat/screen/minicom on /dev/ttyUSB0 while the router
#   is up. They steal bytes and produce misleading symptoms.
#
# * Nucleus command port accepts ONE client. Disconnect the Nortek
#   GUI on the laptop before running this.
# ============================================================
set -o pipefail

# ---------------- config ----------------
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/nautilus_ws/install/setup.bash"

# Vehicle (Cube / MAVROS)
FCU_DEV="/dev/ttyUSB0"
FCU_URL="udp://127.0.0.1:14555@127.0.0.1:14540"   # bind 14555 -> router 14540
TGT_SYS=1
TGT_COMP=1
ROUTER_CONF="/etc/mavlink-router/main.conf"
ROUTER_LOG="/tmp/mavlink_router.log"
MAVROS_LOG="/tmp/mavros_node.log"
ROUTER_SETTLE=8            # let router establish UART link before MAVROS
STATE_TIMEOUT=45           # seconds to wait for connected:true

# Sensor (Nucleus)
NUCLEUS_HOST="192.168.2.201"
NUCLEUS_PW="nortek"
NODE_LOG="/tmp/nucleus_node.log"
CONNECT_RETRIES=3

info() { echo -e "\033[1;32m[stack]\033[0m $*"; }
sub()  { echo -e "\033[1;36m  [$1]\033[0m $2"; }
warn() { echo -e "\033[1;33m[stack]\033[0m $*"; }
err()  { echo -e "\033[1;31m[stack]\033[0m $*"; }

# ---------------- source ROS ----------------
[ -f "$ROS_SETUP" ] || { echo "Missing $ROS_SETUP"; exit 1; }
source "$ROS_SETUP"
[ -f "$WS_SETUP" ] || { echo "Missing $WS_SETUP - build the workspace first"; exit 1; }
source "$WS_SETUP"

# ---------- clock sanity (RTC installed; warn-only) ----------
if [ "$(date +%Y)" -lt 2026 ]; then
  warn "Clock reads $(date) - RTC may have failed."
  warn "If TLS/timestamps misbehave:  sudo date -s \"YYYY-MM-DD HH:MM:SS\""
fi

# ---------------- cleanup (hard) ----------------
CLEANED=false
cleanup() {
  [ "$CLEANED" = true ] && return; CLEANED=true
  echo; info "Tearing down stack..."
  # With SKIP_NUCLEUS there is no nucleus_node to answer this RPC and
  # `ros2 service call` would HANG forever (systemd then stop-timeouts at 90s
  # and marks the run Failed). Skip it; time-bound it even in the normal case.
  if [ "${SKIP_NUCLEUS:-0}" != "1" ]; then
    timeout 3 ros2 service call /nucleus_node/stop interfaces/srv/Stop "{}" >/dev/null 2>&1 || true
    sleep 0.5
  fi
  # pkill -f, not kill $PID: `ros2 run` wrapper dies but the node survives,
  # squatting on the UDP port and breaking the next run.
  pkill -9 -f nucleus_node   2>/dev/null || true
  pkill -9 -f mavros_node    2>/dev/null || true
  pkill -9 mavlink-routerd   2>/dev/null || true
  info "Done."
}
trap cleanup INT TERM EXIT

# ---------------- pre-flight ----------------
info "Pre-flight checks..."

[ -e "$FCU_DEV" ] || { err "$FCU_DEV not found. CP2102 plugged in?  (ls -l /dev/ttyUSB*)"; exit 1; }

if [ -f "$ROUTER_CONF" ]; then
  grep -qi "Mode=Server" "$ROUTER_CONF" || {
    err "$ROUTER_CONF has no Mode=Server endpoint. MAVROS will not connect."
    err "Add:  [UdpEndpoint mavros] / Mode=Server / Address=0.0.0.0 / Port=14540"
    exit 1; }
  dupes=$(grep -c "^\[UartEndpoint" "$ROUTER_CONF")
  [ "$dupes" -gt 1 ] && { err "$ROUTER_CONF has $dupes UartEndpoint blocks - UART will double-open."; exit 1; }
else
  warn "$ROUTER_CONF not found - router will use defaults."
fi

warn "Nortek Nucleus GUI on the laptop must be DISCONNECTED (command port = 1 client)."

info "Clearing stray processes..."
pkill -9 -f nucleus_node 2>/dev/null || true
pkill -9 -f mavros_node  2>/dev/null || true
pkill -9 mavlink-routerd 2>/dev/null || true
sleep 2

for p in 14540 14555; do
  if ss -ulpn 2>/dev/null | grep -q ":$p "; then
    err "UDP port $p still in use after cleanup. Check: sudo ss -ulpn | grep $p"
    exit 1
  fi
done
sub preflight "Ports 14540/14555 free, $FCU_DEV present."

# ================= 1. ROUTER =================
sub router "Starting mavlink-router (log: $ROUTER_LOG)..."
mavlink-routerd >"$ROUTER_LOG" 2>&1 &
sleep 2
pgrep -x mavlink-routerd >/dev/null || { err "Router failed - see $ROUTER_LOG"; cat "$ROUTER_LOG"; exit 1; }

opens=$(grep -c "Opened UART.*${FCU_DEV}" "$ROUTER_LOG" 2>/dev/null || echo 0)
[ "${opens:-0}" -eq 0 ] && { err "Router never opened $FCU_DEV - see $ROUTER_LOG"; exit 1; }
[ "${opens:-0}" -gt 1 ] && { err "$FCU_DEV opened $opens times (double-open)."; exit 1; }
grep -q "UDP Server.*14540" "$ROUTER_LOG" || warn "No 'UDP Server ... 14540' in router log - check Mode=Server."

sub router "Up. $FCU_DEV opened once. Settling ${ROUTER_SETTLE}s..."
sleep "$ROUTER_SETTLE"

# ================= 2. MAVROS =================
sub mavros "Starting MAVROS (log: $MAVROS_LOG)..."
sub mavros "fcu_url = $FCU_URL"
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="$FCU_URL" \
  -p tgt_system:=$TGT_SYS -p tgt_component:=$TGT_COMP \
  >"$MAVROS_LOG" 2>&1 &

sub mavros "Waiting for connected:true (up to ${STATE_TIMEOUT}s)..."
connected=false; state=""
for i in $(seq 1 "$STATE_TIMEOUT"); do
  pgrep -f mavros_node >/dev/null || { err "MAVROS died - see $MAVROS_LOG"; tail -20 "$MAVROS_LOG"; exit 1; }
  if grep -q "Address already in use" "$MAVROS_LOG" 2>/dev/null; then
    err "UDP bind conflict. fcu_url must NOT bind the router's port (14540)."
    err "Correct form: udp://127.0.0.1:14555@127.0.0.1:14540"
    exit 1
  fi
  state=$(timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null)
  echo "$state" | grep -q "connected: true" && { connected=true; break; }
  sleep 1
done

if [ "$connected" = true ]; then
  mode=$(echo "$state" | grep "mode:" | head -1 | awk '{print $2}')
  sub mavros "CONNECTED. FCU mode: ${mode:-unknown}"
else
  err "MAVROS not connected after ${STATE_TIMEOUT}s."
  err "Check: Cube powered from battery? ArduSub booted? TELEM2 harness (3 wires, no 5V)?"
  err "Verify Cube is streaming:  pkill -9 mavlink-routerd; timeout 3 cat $FCU_DEV | xxd | head"
  err "Log: $MAVROS_LOG"
  exit 1
fi

# ================= 2b. STREAM RATES =================
# FCU defaults to ~10Hz on the attitude/IMU stream. Heading PID wants ~50Hz.
# This is TRANSIENT - it does not survive an FCU reboot. The persistent fix is
# SRx_EXTRA1=50 on whichever SERIALx carries MAVLink to the Jetson.
sub mavros "Requesting 50Hz stream rate..."
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
  "{stream_id: 0, message_rate: 50, on_off: true}" >/dev/null 2>&1 \
  || warn "set_stream_rate call failed - IMU may stay at ~10Hz."
sleep 1

# `ros2 topic hz` never exits on its own - timeout is mandatory here.
imu_hz=$(timeout 5 ros2 topic hz /mavros/imu/data 2>/dev/null \
         | grep -o 'average rate: [0-9.]*' | tail -1 | awk '{print $3}')
if [ -n "$imu_hz" ] && [ "${imu_hz%.*}" -gt 30 ] 2>/dev/null; then
  sub mavros "IMU at ${imu_hz} Hz."
else
  warn "IMU at ${imu_hz:-unknown} Hz - expected ~50. Check SERIALx_PROTOCOL / SRx_EXTRA1."
fi

# ================= 3. NUCLEUS =================
# SKIP_NUCLEUS=1 brings up router + MAVROS only (no DVL/INS). For the
# open-loop hard-code run, which needs only the Cube -- use it when the
# Nucleus won't connect and you don't need heading/position.
if [ "${SKIP_NUCLEUS:-0}" = "1" ]; then
  warn "SKIP_NUCLEUS=1: bringing up router + MAVROS ONLY, no Nucleus/DVL/INS."
else
sub nucleus "Starting nucleus_node (log: $NODE_LOG)..."
ros2 run nucleus_driver_ros2 nucleus_node >"$NODE_LOG" 2>&1 &

sub nucleus "Waiting for services..."
svc=false
for i in $(seq 1 30); do
  pgrep -f nucleus_node >/dev/null || { err "nucleus_node died - see $NODE_LOG"; tail -20 "$NODE_LOG"; exit 1; }
  ros2 service list 2>/dev/null | grep -q "/nucleus_node/connect_tcp" && { svc=true; break; }
  sleep 0.5
done
[ "$svc" = true ] || { err "nucleus_node services never appeared - see $NODE_LOG"; exit 1; }

nuc=false
for attempt in $(seq 1 "$CONNECT_RETRIES"); do
  sub nucleus "Connecting to $NUCLEUS_HOST (attempt $attempt/$CONNECT_RETRIES)..."
  out=$(ros2 service call /nucleus_node/connect_tcp interfaces/srv/ConnectTcp \
        "{host: '$NUCLEUS_HOST', password: '$NUCLEUS_PW'}" 2>&1)
  echo "$out" | grep -q "status=True" && { nuc=true; break; }
  echo "$out" | grep -qi "too many connections" && sub nucleus "Instrument says 'Too many connections' - close the Nortek GUI."
  sleep 2
done
[ "$nuc" = true ] || { err "Nucleus connect failed. GUI disconnected? ping $NUCLEUS_HOST?"; exit 1; }
sub nucleus "Connected."

out=$(ros2 service call /nucleus_node/start interfaces/srv/Start "{}" 2>&1)
echo "$out" | grep -q "reply='OK" || { err "Nucleus start failed:"; echo "$out"; exit 1; }
sub nucleus "STREAMING."
fi

# ================= SUMMARY =================
echo
info "================= NAUTILUS STACK UP ================="
info "Vehicle : /mavros/state  /mavros/imu/data  /mavros/rc/*"
info "          /mavros/cmd/arming  /mavros/set_mode"
info "Sensor  : /nucleus_node/ins_packets          (heading, attitude, position)"
info "          /nucleus_node/bottom_track_packets (DVL velocity - needs water)"
info "          /nucleus_node/altimeter_packets    (altitude above bottom)"
info "===================================================="
info "Verify in a new terminal:"
echo "    source /opt/ros/humble/setup.bash && source ~/nautilus_ws/install/setup.bash"
echo "    ros2 topic echo /mavros/state --once"
echo "    ros2 topic echo /nucleus_node/ins_packets"
echo
info "Logs: $ROUTER_LOG  $MAVROS_LOG  $NODE_LOG"
info "Ctrl+C to tear down the whole stack."

wait
