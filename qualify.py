#!/usr/bin/env python3
"""
qualify.py - Nautilus gate run.

    Look for the gate -> drive to it -> stop -> disarm

ALMOST NO DEPTH LOGIC. Mode is ALT_HOLD everywhere except the ONE
deliberate exception, SEEK_ALTITUDE, right after arming: ALT_HOLD's z
channel is a capped climb/descent RATE around a hold point (the mode's job
is to resist depth change), too gentle for actively driving to a target
depth. So SEEK_ALTITUDE switches to STABILIZE (direct z response, still
levels roll/pitch), reads /nucleus_node/altimeter_packets and drives z
until the vehicle is TARGET_ALTITUDE_M above the pool floor, then switches
back to ALT_HOLD before anything else runs. Every other phase never
touches z, and ALT_HOLD does the actual holding for the rest of the run.

SEEK_ALTITUDE's z-direction (altitude_sign param, default ALTITUDE_SIGN) is
UNVERIFIED -- whether raising z moves the vehicle up or down is an
ArduSub/wiring convention this code does not know. Watch the first run with
the vehicle well clear of the floor; if altimeter_distance moves the WRONG
way, pass -p altitude_sign:=-1.0 (no rebuild needed) and rerun.

Two independent backstops guard against a wrong sign actually reaching the
floor. altitude_min_safe_m is a hard abort on the raw altimeter reading,
checked every tick, regardless of which direction was intended or how
aggressive the gains are -- it does NOT depend on altitude_sign being
right, which is exactly why it must be set with real margin for your pool
and never left at 0 (0 never actually trips: the altimeter would have to
read a negative distance, i.e. the vehicle is already through the floor).
Separately, seek_altitude() tracks whether the altitude error is shrinking
or growing over each ALTITUDE_STALL_CHECK_S window; growing while z is well
off neutral is the signature of a backwards sign, and that aborts
immediately instead of continuing to lean into it for the full
ALTITUDE_TIMEOUT.

Vision (pipe_detector.py, /nautilus/detections) owns WHERE the gate is; the
drive phase never terminates on a dead-reckoned distance, only on actually
seeing the gate close up. pipe_detector.py does its own confirm-streak state
tracking (tracking.py) -- a gate is only "confirmed" once several consecutive
frames agree on bearing, and a brief dropout doesn't erase it. This node
trusts "confirmed" at ARM time (the one moment that matters most) and uses
whatever's fresh while driving (a weaker reading is fine once under way --
it's corroborated by everything before it).

The INS owns heading and supplies the sanity bounds that abort the run if we
drive somewhere absurd.

hard_code_enable:=true is the dead-reckoned qualify path -- NO vision, no
gate, no camera. It descends hard_code_down_distance feet below the launch
altitude, then drives hard_code_forward_distance feet forward on the heading
captured at arm, then disarms. It reuses the exact same altimeter descent
loop as SEEK_ALTITUDE (same safety floor, same backwards-sign backstops,
same STABILIZE->ALT_HOLD handoff) and the same INS dead-reckoning
(along_cross) the vision path uses for GATE_OVERSHOOT -- it just aims at
fixed distances instead of at what the camera sees. This is the minimum
needed to qualify (get down, get through) and is what to run when vision
isn't ready. The three toggles are module constants below AND ROS
parameters of the same name, so one CLI command sets them (see
hard_code_run.sh for a one-command wrapper). "Down" is measured as the
altimeter reading shrinking, so on a flat floor it equals a depth increase;
it still depends on the altimeter, so the same altitude_sign / safety-floor
cautions apply.

sim_sensors:=true fabricates FCU/INS/altimeter data too, so the whole state
machine (WAIT_FCU -> ... -> SEEK_ALTITUDE -> SETTLE -> DRIVE_TO_GATE ->
DISARM) can be run above water with nothing else launched -- no MAVROS, no
nucleus_node, no pipe_detector. A free-running timer (_sim_tick) integrates
whatever this node last actually commanded (see publish()) against a fixed,
made-up ground truth (SIM_ALT_TRUTH_SIGN) for which way z moves the
simulated vehicle, and fake_gate (independent flag, turn it on too) covers
vision. This proves the SEQUENCING and the abort/safety LOGIC -- including
that the SEEK_ALTITUDE backstops above actually fire -- end to end on the
bench. It proves NOTHING about which way the real vehicle's z channel
actually moves, or about real gains, real noise, or a real DVL's bottom
lock: only a real pool run with the vehicle clear of the floor verifies
altitude_sign. sim_sensors forces dry_run on; it must never drive a real
vehicle.

    ros2 run nautilus_auto qualify --ros-args -p dry_run:=true
    ros2 run nautilus_auto qualify --ros-args -p fake_gate:=true   # no camera needed
    ros2 run nautilus_auto qualify --ros-args -p target_altitude_m:=0.6 -p altitude_sign:=-1.0
    ros2 run nautilus_auto qualify --ros-args -p sim_sensors:=true -p fake_gate:=true -p fake_gate_range_m:=1.0
        # bench, above water, nothing else running. fake_gate_range_m must be
        # below GATE_PASS_RANGE (1.6) or the simulated gate never closes
        # enough to finish drive_to_gate() -- expect ~15-20s to reach it,
        # that's FAKE_GATE_SECONDS playing out, not a hang.

Every console log line is mirrored to a plain text file (see _open_run_log)
alongside the per-command CSV (see _open_cmd_log), both under CMD_LOG_DIR --
a complete record of the run independent of console log level or terminal
scrollback.

PRE-FLIGHT: confirm in water that velocity_nucleus_x goes nonzero and
fom_ins drops below ~5. On the bench it reads 0.0 / 44.9 and the position
solution is meaningless. Run with require_bottom_lock:=true to hard-gate it.
Also watch raw altimeter_distance against known pool depth for a bit before
trusting SEEK_ALTITUDE at all -- garbage in is garbage out no matter how
altitude_sign and altitude_min_safe_m are set.
"""

import json
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Empty, String
from rosidl_runtime_py.utilities import get_message

INS_TOPIC = "/nucleus_node/ins_packets"
ALTIMETER_TOPIC = "/nucleus_node/altimeter_packets"
Z_NEUTRAL = 500.0   # ManualControl z: 500 = no vertical demand (ALT_HOLD)
DT = 0.05           # 20 Hz control loop
FAKE_GATE_SECONDS = 20.0   # fake_gate:=true fakes a solid gate for this long, once
FAKE_GATE_START_RANGE = 8.0   # m: simulated range at the start of the fake window

# SEEK_ALTITUDE (the one exception to "no depth logic" -- see module docstring)
# target_altitude_m / altitude_sign / altitude_min_safe_m are also exposed as
# ROS parameters of the same name (see __init__) so they can be changed
# poolside without a rebuild -- these constants are just their defaults.
TARGET_ALTITUDE_M = 0.5      # m above the pool floor to reach before SETTLE
ALTITUDE_TOLERANCE_M = 0.1    # m: within this band of target counts as "there"
ALTITUDE_KP = 300.0           # z units per metre of error -- verify in pool
ALTITUDE_Z_MAX = 200.0        # z units off neutral, hard cap (of the 0-1000 range) --
                              # raised from 80: that wasn't moving the vehicle fast
                              # enough. Still short of full-scale (500) on purpose.
ALTITUDE_TIMEOUT = 120.0      # s to reach target before aborting
ALTIMETER_TIMEOUT = 3.0       # s: altimeter reports in pulses, not continuously --
                              # this must be looser than INS_TIMEOUT or every gap
                              # between pulses reads as "stale" and the z command
                              # pulses on/off in lockstep with the sensor.
ALTITUDE_MIN_SAFE_M = 0.3  # m: abort immediately this close to the floor, no matter what.
                           # THE ONE CHECK THAT DOES NOT DEPEND ON altitude_sign BEING
                           # RIGHT. Was 0.0, which never actually tripped (the altimeter
                           # would have to read a negative distance) -- that's how a
                           # prior run drove straight to the pool bottom instead of
                           # stopping. 0.3 is a placeholder: set this from your actual
                           # pool depth / vehicle draft with real margin, and keep it
                           # comfortably below target_altitude_m.
ALTITUDE_SIGN = 1.0           # UNVERIFIED direction -- flip to -1.0 if it moves the wrong way
ALTITUDE_STALL_CHECK_S = 8.0   # s: window to judge "is it actually moving" (and,
                               # now, moving the right way -- see seek_altitude)
ALTITUDE_STALL_MIN_MOVE_M = 0.03   # m: less than this over the window counts as stalled
ALTITUDE_DIVERGE_M = 0.15     # m: if the altitude error gets this much WORSE (not
                              # better) over one ALTITUDE_STALL_CHECK_S window while z
                              # is meaningfully off neutral, that's a backwards
                              # altitude_sign, not noise -- abort rather than continue

FT_TO_M = 0.3048
POOL_DEPTH_FT = 5.0        # ft: THIS TEST's pool depth. Normal/competition pool is
                           # 7 ft -- change this back (or pass -p pool_depth_ft:=7.0)
                           # before a non-test run. Also a ROS parameter (see
                           # __init__). Used only as an upper sanity bound on raw
                           # altimeter readings in _on_altimeter: the vehicle
                           # physically cannot be deeper than the pool it's in, so
                           # anything past that is bad data, not a bad sign/gain.
POOL_DEPTH_SLACK_M = 0.3   # m: margin added on top of pool_depth_m before rejecting
                           # a reading -- the depth figure above is a rough number,
                           # not a survey, and a tilted vehicle reads a bit long.

# sim_sensors (bench testing only -- see module docstring and _sim_tick).
# These calibrate a fake plant so the state machine has something to
# converge against; they are NOT a claim about the real vehicle.
SIM_MAX_SPEED_MPS = 0.5     # m/s of simulated forward speed at full cruise thrust
SIM_MAX_CLIMB_MPS = 0.3     # m/s of simulated altitude change at full commanded z authority
SIM_ALT_TRUTH_SIGN = 1.0    # the simulator's OWN made-up ground truth for which way z
                            # moves the simulated vehicle. Arbitrary, exists only to
                            # give SEEK_ALTITUDE's logic something consistent to
                            # converge (or, with altitude_sign flipped, diverge and
                            # abort) against. Tells you NOTHING about which way the
                            # real vehicle's z channel moves.

# HARD-CODE MODE (dead-reckoned qualify: down then forward, no vision -- see
# module docstring). These three toggles ARE the whole test. They are module
# constants (edit here) AND ROS parameters of the same name (see __init__), so
# one CLI command sets them without editing the file, e.g.:
#   python3 qualify.py --ros-args -p hard_code_enable:=true \
#       -p hard_code_down_distance:=3 -p hard_code_forward_distance:=10
# or just run hard_code_run.sh, which wraps exactly that.
HARD_CODE_ENABLE = False               # master switch. true -> skip vision entirely:
                                       # descend a fixed distance, drive forward a fixed
                                       # distance, disarm. Default false so a normal
                                       # (vision) run is unaffected unless asked for.
HARD_CODE_DOWN_DISTANCE_FT = 3.0       # ft to descend BELOW the launch altitude
HARD_CODE_FORWARD_DISTANCE_FT = 10.0   # ft to drive forward on the captured heading

# Fixed tuning constants. These get set once from pool testing and rarely
# change between runs -- edit them here rather than adding another ROS
# parameter nobody remembers to set. Only the things you'd actually want to
# change per-run without a rebuild are declared as parameters below.
KP_HEADING, KI_HEADING = 6.0, 0.4
YAW_LIMIT, I_LIMIT = 400.0, 200.0
VISION_TIMEOUT = 1.5          # s before a detection goes stale
SETTLE_SECONDS = 3.0          # s of heading hold before driving
GATE_OVERSHOOT = 2.5          # m of blind push once the gate leaves frame
GATE_PASS_RANGE = 1.6         # m: closer than this, commit to passing
GATE_LOST_STOP = 5.0          # s a lost gate may coast before stopping forward thrust
SCAN_YAW = 150.0              # yaw demand while spinning in place looking for the gate
SEARCH_TIMEOUT = 90.0         # s
GATE_MIN_CONF = 0.20          # sanity floor at ARM time only
INS_TIMEOUT = 1.0             # s
STATE_MAX_AGE = 5.0           # s: reject latched/stale /mavros/state
CMD_LOG_DIR = "~/nautilus_ws/logs"


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Qualify(Node):

    # ---------------- setup ----------------

    def __init__(self):
        super().__init__("qualify")
        # Printed straight away, before parameters/topics/log files are even
        # set up, so a dry run (or a bench run with nothing else hooked up)
        # proves the process is alive immediately instead of going silent
        # for however long topic/service waits below take.
        self.get_logger().info("qualify.py started")

        d = self.declare_parameter
        d("dry_run", False)
        d("fake_gate", False)                # bench: fake a solid gate, no camera needed --
                                             # default ON; pass fake_gate:=false for a real run
        d("fake_gate_bearing_deg", 0.0)     # bearing to report while faking
        d("fake_gate_range_m", 3.0)         # range simulated CLOSING to by the end of the window
        d("sim_sensors", False)             # bench: fabricate FCU/INS/altimeter too, no other
                                             # nodes needed (see module docstring). Forces dry_run.
        d("target_heading", float("nan"))   # NaN -> capture at arm
        d("cruise_speed", 0.35)             # thrust fraction while driving
        d("arm_delay", 5.0)                 # s between gate acquired and arming
        d("mission_timeout", 1000.0)        # s, hard abort -- runaway guard, not a schedule
        d("gate_acquire_timeout", 0.0)      # s to wait on deck. 0 = forever.
        d("skip_gate_wait", False)          # bench only: arm without seeing a gate
        d("require_bottom_lock", False)     # hard-gate on DVL bottom lock at launch
        d("max_fom_ins", 10.0)              # only enforced if require_bottom_lock
        d("target_altitude_m", TARGET_ALTITUDE_M)     # m above floor to reach before SETTLE
        d("altitude_sign", ALTITUDE_SIGN)             # UNVERIFIED direction -- see module docstring
        d("altitude_min_safe_m", ALTITUDE_MIN_SAFE_M)  # hard abort floor -- size to the real pool
        d("pool_depth_ft", POOL_DEPTH_FT)             # ft: sanity ceiling for altimeter readings
        d("hard_code_enable", HARD_CODE_ENABLE)               # dead-reckoned down-then-forward, no vision
        d("hard_code_down_distance", HARD_CODE_DOWN_DISTANCE_FT)      # ft to descend below launch altitude
        d("hard_code_forward_distance", HARD_CODE_FORWARD_DISTANCE_FT)  # ft to drive forward on captured heading

        g = lambda n: self.get_parameter(n).value
        self.dry_run = bool(g("dry_run"))
        self.fake_gate = bool(g("fake_gate"))
        self.fake_bearing = float(g("fake_gate_bearing_deg"))
        self.fake_range = float(g("fake_gate_range_m"))
        self.sim_sensors = bool(g("sim_sensors"))
        if self.sim_sensors and not self.dry_run:
            self.dry_run = True   # sim_sensors must never drive a real vehicle -- warning logged below
        self.gate_heading = float(g("target_heading"))
        self.cruise_v = float(g("cruise_speed")) * 1000.0
        self.arm_delay = float(g("arm_delay"))
        self.mission_timeout = float(g("mission_timeout"))
        self.acquire_timeout = float(g("gate_acquire_timeout"))
        self.require_lock = bool(g("require_bottom_lock"))
        self.max_fom = float(g("max_fom_ins"))
        self.target_altitude_m = float(g("target_altitude_m"))
        self.altitude_sign = float(g("altitude_sign"))
        self.altitude_min_safe_m = float(g("altitude_min_safe_m"))
        self.pool_depth_m = float(g("pool_depth_ft")) * FT_TO_M
        self.hard_code = bool(g("hard_code_enable"))
        self.hard_down_m = float(g("hard_code_down_distance")) * FT_TO_M
        self.hard_forward_m = float(g("hard_code_forward_distance")) * FT_TO_M
        self._fake_until = None   # lazily set on first gate check, not at startup

        # INS state. Under sim_sensors these are seeded live (not None/0)
        # since no real _on_ins will ever arrive to populate them, and kept
        # fresh afterward by the _sim_tick timer set up below.
        self.heading = 0.0 if self.sim_sensors else None
        self.pos = (0.0, 0.0) if self.sim_sensors else None   # (x, y) in INS frame
        self.vel_x = 0.0
        self.fom_ins = 1.0 if self.sim_sensors else 999.0
        self.ins_stamp = time.time() if self.sim_sensors else 0.0

        # Altimeter state (SEEK_ALTITUDE only). Seeded near the surface under
        # sim_sensors (pool depth minus a little headroom, not pool depth
        # exactly) -- i.e. the simulated vehicle starts close to the top of
        # the water column. The headroom matters: seeded AT the ceiling,
        # _sim_tick's clamp would pin a wrong-direction test flat instead of
        # showing the error actually growing, masking the exact divergence
        # the ALTITUDE_DIVERGE_M check exists to catch.
        self.altimeter_distance = max(0.0, self.pool_depth_m - 0.5) if self.sim_sensors else None
        self.altimeter_stamp = time.time() if self.sim_sensors else 0.0

        # Mission frame: origin + axis latched at arm
        self.origin = None
        self.armed_by_us = False
        self.phase = "INIT"
        self.mission_start = None

        # Heading PI
        self.integral = 0.0
        self.last_t = None

        # Vision: the pipe_detector.py "gate" dict plus recv_t, or None.
        # pipe_detector.py already does confirm-streak and hold-through-miss
        # (age_s); recv_t catches the case where pipe_detector itself has
        # died and stopped publishing entirely.
        self._gate = None
        self._log_times = {}

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self._cmd_log = self._open_cmd_log()
        self._run_log = self._open_run_log()

        if not self.sim_sensors:
            self.create_subscription(State, "/mavros/state", self._on_state,
                                     self._match_qos("/mavros/state"))
        self.ctrl = self.create_publisher(ManualControl,
                                          "/mavros/manual_control/send", 10)
        if not self.sim_sensors:
            self._subscribe_ins(qos)
            self._subscribe_altimeter(qos)
        self.create_subscription(String, "/nautilus/detections",
                                 self._on_detections, 10)
        self.create_subscription(Empty, "/nautilus/cmd/abort",
                                 self._on_abort, 10)
        self._abort_req = False

        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.state = None
        self.state_stamp = 0.0

        # sim_sensors: a free-running timer, not tied to any particular
        # phase's tick() calls, so INS/altimeter stay fresh no matter which
        # blocking wait loop is currently spinning (see _sim_tick).
        self._last_cmd = (0.0, 0.0, Z_NEUTRAL)
        if self.sim_sensors:
            self._sim_last_t = time.time()
            self.create_timer(DT, self._sim_tick)

        if self.dry_run:
            self._log("warn", "DRY RUN: no arm, no thrust published.")
        if self.hard_code:
            self._log("warn",
                f"HARD_CODE MODE: no vision, no gate. Descend "
                f"{self.hard_down_m / FT_TO_M:.1f}ft ({self.hard_down_m:.2f}m) below launch "
                f"altitude, then drive forward {self.hard_forward_m / FT_TO_M:.1f}ft "
                f"({self.hard_forward_m:.2f}m) on the captured heading, then disarm.")
        if self.sim_sensors:
            self._log("warn",
                "SIM_SENSORS: FCU/INS/altimeter are all synthetic, integrated from "
                "this node's own commands against a made-up ground truth -- no "
                "MAVROS/nucleus_node/pipe_detector needed. This exercises the STATE "
                "MACHINE and the abort/safety LOGIC end to end; it proves NOTHING "
                "about altitude_sign or any other real-world direction or gain. "
                "Only a real pool run with the vehicle clear of the floor verifies "
                "that.")
        if self.fake_gate:
            self._log("warn",
                f"FAKE_GATE: gate faked CONFIRMED at bearing {self.fake_bearing:+.1f} deg, "
                f"range simulated closing {FAKE_GATE_START_RANGE:.1f}m -> "
                f"{self.fake_range:.1f}m over {FAKE_GATE_SECONDS:.0f}s, "
                f"starting the first time it's checked.")

    # ---------------- logging ----------------

    def _open_cmd_log(self):
        """Every command this node ever sends (or would send, in dry_run)
        goes here as it's published -- x/y/z/r, post-clamp, with the phase
        active at the time. Plain always-on file, independent of console
        log level or whether a bag was recording /mavros/manual_control/send.
        """
        path = os.path.join(os.path.expanduser(CMD_LOG_DIR),
                            f"qualify_cmds_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        f = open(path, "w", buffering=1)   # line-buffered: survives a crash
        f.write("t,phase,x,y,z,r\n")
        self.get_logger().info(f"command log: {path}")
        return f

    def _open_run_log(self):
        """Mirror of every get_logger() call this node makes, as plain text,
        so the full run is on disk regardless of console log level or
        terminal scrollback -- see _log()."""
        path = os.path.join(os.path.expanduser(CMD_LOG_DIR),
                            f"qualify_run_{time.strftime('%Y%m%d_%H%M%S')}.log")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        f = open(path, "w", buffering=1)
        self.get_logger().info(f"run log: {path}")
        return f

    def _log(self, level, msg):
        """Log through ROS (console) AND the plain-text run log file.

        Each severity gets its own get_logger() call site (not one shared
        `getattr(...)` line) -- rclpy identifies a logging statement by its
        file+line to support throttle/once logging, so funnelling every
        severity through a single line makes it see "the same statement
        used a different severity" and raise ValueError.
        """
        logger = self.get_logger()
        if level == "info":
            logger.info(msg)
        elif level == "warn":
            logger.warn(msg)
        elif level == "error":
            logger.error(msg)
        else:
            logger.fatal(msg)
        self._run_log.write(f"{time.time():.3f} [{level.upper():5s}] {self.phase:16s} {msg}\n")

    def _log_cmd(self, x, y, z, r):
        self._cmd_log.write(
            f"{time.time():.3f},{self.phase},{x:.1f},{y:.1f},{z:.1f},{r:.1f}\n")

    def close_logs(self):
        for f in (self._cmd_log, self._run_log):
            try:
                f.close()
            except Exception:  # noqa: BLE001
                pass

    def _match_qos(self, topic, timeout=8.0):
        """Mirror the publisher's QoS. A mismatched subscriber gets nothing,
        silently, forever -- which reads as "FCU never connected"."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            infos = self.get_publishers_info_by_topic(topic)
            if infos:
                q = infos[0].qos_profile
                self._log("info",
                    f"{topic}: matching pub QoS "
                    f"{q.reliability.name}/{q.durability.name}")
                return QoSProfile(
                    reliability=q.reliability,
                    durability=q.durability,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10)
            rclpy.spin_once(self, timeout_sec=0.2)
        self._log("warn", f"{topic}: no publisher, using default QoS")
        return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

    def _subscribe_ins(self, qos):
        deadline = time.time() + 10.0
        mt = None
        while time.time() < deadline and not mt:
            for name, types in self.get_topic_names_and_types():
                if name == INS_TOPIC and types:
                    mt = types[0]
                    break
            rclpy.spin_once(self, timeout_sec=0.2)
        if not mt:
            self._log("fatal", f"{INS_TOPIC} absent. nautilus_up.sh running?")
            raise SystemExit(1)
        self._log("info", f"INS type: {mt}")
        self.create_subscription(get_message(mt), INS_TOPIC, self._on_ins, qos)

    def _subscribe_altimeter(self, qos):
        deadline = time.time() + 10.0
        mt = None
        while time.time() < deadline and not mt:
            for name, types in self.get_topic_names_and_types():
                if name == ALTIMETER_TOPIC and types:
                    mt = types[0]
                    break
            rclpy.spin_once(self, timeout_sec=0.2)
        if not mt:
            self._log("fatal", f"{ALTIMETER_TOPIC} absent. nautilus_up.sh running?")
            raise SystemExit(1)
        self._log("info", f"altimeter type: {mt}")
        self.create_subscription(get_message(mt), ALTIMETER_TOPIC, self._on_altimeter, qos)

    # ---------------- callbacks ----------------

    def _on_state(self, msg):
        self.state = msg
        self.state_stamp = time.time()

    def _on_abort(self, _msg):
        """Convenience kill, NOT a safety system -- the hardware kill switch
        is. ros2 topic pub --once /nautilus/cmd/abort std_msgs/msg/Empty {}"""
        self._log("warn", "ABORT REQUESTED on /nautilus/cmd/abort")
        self._abort_req = True

    def fcu_live(self):
        """connected:true AND recent. /mavros/state is latched, so a stale
        "connected:true" from a dead MAVROS must not be trusted."""
        if self.sim_sensors:
            return True
        if self.state is None or not self.state.connected:
            return False
        hdr = self.state.header.stamp
        age = self.get_clock().now().nanoseconds * 1e-9 - (
            hdr.sec + hdr.nanosec * 1e-9)
        return age <= STATE_MAX_AGE

    def _on_ins(self, m):
        self.heading = float(m.heading) % 360.0
        self.pos = (float(m.position_frame_x), float(m.position_frame_y))
        self.vel_x = float(m.velocity_nucleus_x)
        self.fom_ins = float(m.fom_ins)
        self.ins_stamp = time.time()

    def _on_altimeter(self, m):
        d = float(m.altimeter_distance)
        if not math.isfinite(d) or d < 0.0 or d > self.pool_depth_m + POOL_DEPTH_SLACK_M:
            # Drop it and leave altimeter_distance/stamp untouched -- a bad
            # sample (or a run of them) just ages into the existing "stale,
            # hold neutral" handling in seek_altitude() instead of being
            # acted on directly. Not confident the nucleus is clean, so
            # don't trust a single reading blindly -- and it physically
            # cannot read deeper than the pool it's in.
            self.log_every("altimeter_bad", 2.0, lambda: (
                f"  ignoring implausible altimeter reading: {d:.2f}m "
                f"(pool_depth_m {self.pool_depth_m:.2f})"))
            return
        self.altimeter_distance = d
        self.altimeter_stamp = time.time()

    def _sim_tick(self):
        """sim_sensors only (see module docstring and __init__). A
        free-running timer, not tied to any particular phase's tick()
        calls, so INS/altimeter stay fresh no matter which blocking wait
        loop (WAIT_FCU, WAIT_GATE, ...) happens to be spinning right now --
        anything hooked only into tick() would go stale during those.

        Integrates whatever this node last actually commanded (_last_cmd,
        set in publish()) into fake position/altitude. Heading is left
        alone -- no yaw dynamics are modeled, since nothing this is meant
        to exercise (phase sequencing, SEEK_ALTITUDE convergence/abort,
        forward progress toward a faked gate) needs the vehicle to
        actually steer.
        """
        now = time.time()
        dt = now - self._sim_last_t
        self._sim_last_t = now
        if dt <= 0.0:
            return

        x, _r, z = self._last_cmd   # yaw (_r) intentionally unused -- no yaw dynamics modeled

        speed = (x / 1000.0) * SIM_MAX_SPEED_MPS
        self.vel_x = speed
        h = math.radians(self.heading)
        px, py = self.pos
        self.pos = (px + speed * math.cos(h) * dt, py + speed * math.sin(h) * dt)

        # Made-up ground truth, NOT a claim about the real vehicle -- see
        # SIM_ALT_TRUTH_SIGN. Gives SEEK_ALTITUDE something to converge on
        # when altitude_sign matches it, and diverge on (triggering the
        # ALTITUDE_DIVERGE_M abort) when it doesn't.
        rate = SIM_ALT_TRUTH_SIGN * (z - Z_NEUTRAL) / ALTITUDE_Z_MAX * SIM_MAX_CLIMB_MPS
        self.altimeter_distance = clamp(self.altimeter_distance + rate * dt,
                                        0.0, self.pool_depth_m)
        self.altimeter_stamp = now

        self.fom_ins = 1.0
        self.ins_stamp = now

    def _on_detections(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if d.get("gate"):
            self._gate = dict(d["gate"], recv_t=time.time())

    def _gate_record(self):
        """The gate reading to use right now: fake_gate override (once, for
        FAKE_GATE_SECONDS, starting the first time this is called -- not at
        node startup, which could be tens of seconds before anyone cares)
        or whatever's actually fresh from pipe_detector.py.

        The faked range SIMULATES CLOSING from FAKE_GATE_START_RANGE down to
        fake_gate_range_m over the window, rather than reporting the
        configured range immediately. Reporting it immediately meant setting
        fake_gate_range_m below GATE_PASS_RANGE made drive_to_gate() see
        "close enough" on its very first tick and finish instantly -- it
        never actually held/drove for the configured duration.
        """
        if self.fake_gate:
            if self._fake_until is None:
                self._fake_until = time.time() + FAKE_GATE_SECONDS
            remaining = self._fake_until - time.time()
            if remaining > 0:
                frac = 1.0 - (remaining / FAKE_GATE_SECONDS)
                start = max(FAKE_GATE_START_RANGE, self.fake_range)
                sim_range = start + (self.fake_range - start) * frac
                return {"bearing_deg": self.fake_bearing, "range_m": round(sim_range, 2),
                       "range_ok": True, "conf": 1.0, "confirmed": True}

        if not self._gate:
            return None
        total_age = self._gate["age_s"] + (time.time() - self._gate["recv_t"])
        return self._gate if total_age < VISION_TIMEOUT else None

    def gate_bearing(self):
        r = self._gate_record()
        return r["bearing_deg"] if r else None

    def gate_range(self):
        """(range_m, ok) or (None, False) when no fresh detection."""
        r = self._gate_record()
        return (r["range_m"], r["range_ok"]) if r else (None, False)

    def gate_conf(self):
        r = self._gate_record()
        return r["conf"] if r else 0.0

    def gate_is_solid(self):
        """A gate good enough to ARM on. pipe_detector.py already requires
        several consecutive agreeing frames before it marks "confirmed";
        this just adds a confidence floor on top, and is ONLY applied at
        acquisition. Once driving, a weaker reading is fine -- it's
        corroborated by everything before it."""
        r = self._gate_record()
        return bool(r and r.get("confirmed") and r["conf"] >= GATE_MIN_CONF)

    # ---------------- geometry ----------------

    def along_cross(self):
        """Position projected onto the gate axis: (along, cross) in metres.
        `along` is distance past the gate along the run heading; `cross` is
        lateral offset, +right."""
        dx = self.pos[0] - self.origin[0]
        dy = self.pos[1] - self.origin[1]
        h = math.radians(self.gate_heading)
        along = dx * math.cos(h) + dy * math.sin(h)
        cross = -dx * math.sin(h) + dy * math.cos(h)
        return along, cross

    def ins_ok(self):
        if self.heading is None or self.pos is None:
            return False
        if (time.time() - self.ins_stamp) > INS_TIMEOUT:
            return False
        if self.require_lock and self.fom_ins > self.max_fom:
            return False
        return True

    # ---------------- actuation ----------------

    def publish(self, x, r, z=Z_NEUTRAL, y=0.0):
        m = ManualControl()
        m.header.stamp = self.get_clock().now().to_msg()
        m.x = float(clamp(x, -1000, 1000))
        m.y = float(clamp(y, -1000, 1000))
        m.z = float(clamp(z, 0, 1000))
        m.r = float(clamp(r, -1000, 1000))
        m.buttons = 0
        self._log_cmd(m.x, m.y, m.z, m.r)   # log even in dry_run: what WOULD move
        self._last_cmd = (m.x, m.r, m.z)   # sim_sensors' _sim_tick integrates this
        if not self.dry_run:
            self.ctrl.publish(m)

    def neutral(self):
        self.publish(0.0, 0.0)

    def yaw_to(self, target_hdg):
        """PI on heading error. Caller must have checked ins_ok()."""
        err = wrap180(target_hdg - self.heading)
        now = time.time()
        dt = 0.0 if self.last_t is None else now - self.last_t
        self.last_t = now
        if dt > 0.0:
            self.integral = clamp(self.integral + err * dt, -I_LIMIT, I_LIMIT)
        return clamp(KP_HEADING * err + KI_HEADING * self.integral,
                     -YAW_LIMIT, YAW_LIMIT)

    def reset_pi(self):
        self.integral = 0.0
        self.last_t = None

    # ---------------- services ----------------

    def _call(self, cli, req, what):
        if not cli.wait_for_service(timeout_sec=5.0):
            self._log("error", f"{what}: service unavailable")
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def arm(self, value):
        if self.dry_run:
            self._log("info", f"[dry_run] arm({value})")
            return True
        req = CommandBool.Request()
        req.value = value
        r = self._call(self.arm_cli, req, "arm")
        ok = bool(r and r.success)
        if ok and value:
            self.armed_by_us = True
        return ok

    def set_mode(self, mode):
        if self.dry_run:
            self._log("info", f"[dry_run] set_mode({mode})")
            return True
        req = SetMode.Request()
        req.custom_mode = mode
        r = self._call(self.mode_cli, req, "set_mode")
        return bool(r and r.mode_sent)

    # ---------------- loop primitives ----------------

    def guard(self):
        """Returns None if all is well, else an abort reason."""
        if self._abort_req:
            return "abort requested"
        if not self.ins_ok():
            return f"INS unusable (stale, or fom_ins={self.fom_ins:.1f})"
        if self.mission_start and \
                time.time() - self.mission_start > self.mission_timeout:
            return "mission timeout"
        return None

    def tick(self, x, r, z=Z_NEUTRAL, y=0.0):
        """One control iteration. Raises Abort on guard failure."""
        rclpy.spin_once(self, timeout_sec=0.0)
        why = self.guard()
        if why:
            raise Abort(why)
        self.publish(x, r, z, y)
        time.sleep(DT)

    def spin_until(self, pred, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
        self._log("error", f"timeout waiting for {what}")
        return False

    def enter(self, phase):
        self.phase = phase
        self.reset_pi()
        self._log("info", f"=== {phase} ===")

    def log_every(self, key, interval, msg):
        """Throttled info log, shared by every phase's progress line."""
        now = time.time()
        if now - self._log_times.get(key, 0.0) > interval:
            self._log_times[key] = now
            self._log("info", msg())

    # ---------------- mission ----------------

    def run(self):
        self.enter("WAIT_FCU")
        if not self.spin_until(self.fcu_live, 30.0, "a fresh connected:true"):
            if self.state is None:
                self._log("error",
                    "no /mavros/state at all. Is MAVROS running? "
                    "pgrep -af mavros_node")
            elif not self.state.connected:
                self._log("error",
                    "MAVROS up, FCU silent. Cube powered? TELEM2 harness?")
            else:
                self._log("error",
                    "connected:true but STALE - that is a latched message "
                    "from a dead MAVROS. Check: ros2 topic hz /mavros/state")
            return 1

        self.enter("WAIT_INS")
        if not self.spin_until(lambda: self.pos is not None, 20.0, "INS"):
            return 1

        self._log("info",
            f"hdg {self.heading:.1f}  fom_ins {self.fom_ins:.1f}  "
            f"vel_x {self.vel_x:.3f}  pos {self.pos[0]:.2f},{self.pos[1]:.2f}")
        if self.fom_ins > self.max_fom:
            msg = (f"fom_ins={self.fom_ins:.1f} > {self.max_fom} - no bottom "
                   f"lock. Dead reckoning will drift.")
            if self.require_lock:
                self._log("fatal", msg)
                return 1
            self._log("warn", msg + " Proceeding anyway.")

        if self.hard_code:
            # No vision. wait_for_gate() is where mission_start is stamped and
            # where the hands-off arm countdown lives; hard-code skips the gate
            # wait but keeps the countdown -- arming with no warning window is
            # the one thing the vision path is careful never to do.
            self._log("warn",
                f"HARD_CODE: no gate wait. ARMING IN {self.arm_delay:.0f}s -- HANDS OFF.")
            t_end = time.time() + self.arm_delay
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._abort_req:
                    self._log("error", "Aborted during arm countdown.")
                    return 1
                remain = t_end - time.time()
                if remain > 0 and abs(remain - round(remain)) < 0.06:
                    self._log("warn", f"  arming in {round(remain)}...")
            self.mission_start = time.time()
        elif not self.wait_for_gate():
            return 1

        if math.isnan(self.gate_heading):
            self.gate_heading = self.heading
            self._log("info", f"Run heading CAPTURED: {self.gate_heading:.1f}")
        self.origin = self.pos

        self.enter("ARM")
        self.neutral()
        if not self.arm(True):
            self._log("fatal", "arm rejected (SYSID_MYGCS=1?)")
            return 1
        # STABILIZE for SEEK_ALTITUDE: ALT_HOLD's z channel is a capped
        # climb/descent RATE around a hold point (the mode's whole job is
        # to resist depth change) -- too gentle for actively driving to a
        # target depth. STABILIZE gives z a much more direct thruster
        # response while still levelling roll/pitch. seek_altitude()
        # switches back to ALT_HOLD itself once it reaches target, before
        # anything else runs.
        if not self.set_mode("STABILIZE"):
            self._log("fatal", "mode rejected")
            self.arm(False)
            return 1

        try:
            if self.hard_code:
                self._prepare_hard_code_descent()   # sets target_altitude_m from launch alt
                self.seek_altitude()                # same descent loop + safety as vision path
                self.settle()
                self.hard_code_forward()            # dead-reckoned forward, no vision
            else:
                self.seek_altitude()
                self.settle()
                self.drive_to_gate()
        except Abort as e:
            return self.abort(str(e))

        self.enter("DISARM")
        for _ in range(20):
            self.neutral()
            rclpy.spin_once(self, timeout_sec=DT)
        self.arm(False)
        self._log("info", "Run complete.")
        return 0

    def wait_for_gate(self):
        """Hold on the surface, disarmed, until pipe_detector.py reports a
        confirmed gate (or fake_gate fakes one). The run does not begin on
        an enter keypress -- it begins when the vehicle can see what it's
        aiming at. skip_gate_wait bypasses this on the bench."""
        self.enter("WAIT_GATE")
        if bool(self.get_parameter("skip_gate_wait").value):
            self._log("warn", "skip_gate_wait: arming without a gate.")
            self.mission_start = time.time()
            return True

        forever = self.acquire_timeout <= 0.0
        deadline = None if forever else time.time() + self.acquire_timeout
        if forever:
            self._log("info",
                "Waiting indefinitely for the gate. Disarmed, no thrust. "
                "Ctrl+C or /nautilus/cmd/abort to stop.")

        while rclpy.ok():
            if self._abort_req:
                self._log("error", "Aborted before arming.")
                return False
            if deadline and time.time() > deadline:
                self._log("error", "Gate never appeared. Not arming.")
                return False

            rclpy.spin_once(self, timeout_sec=0.05)
            if self.ins_ok() and self.gate_is_solid():
                break

            def _status():
                left = "" if forever else f"  ({deadline - time.time():.0f}s left)"
                b = self.gate_bearing()
                rng, ok = self.gate_range()
                why = "none" if b is None else f"weak (conf {self.gate_conf():.2f})"
                return (f"  waiting for gate [{why}]  bearing {b}  "
                       f"range {rng if ok else '--'}  hdg {self.heading:.1f}  "
                       f"fom {self.fom_ins:.1f}{left}")
            self.log_every("wait_gate", 5.0, _status)
        else:
            return False

        b = self.gate_bearing()
        rng, ok = self.gate_range()
        self._log("warn",
            f"GATE ACQUIRED: bearing {b:+.1f}, range {rng if ok else 'far'}. "
            f"ARMING IN {self.arm_delay:.0f}s -- HANDS OFF.")

        # Countdown. A confirmed gate is not consent to arm instantly.
        t_end = time.time() + self.arm_delay
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._abort_req:
                self._log("error", "Aborted during arm countdown.")
                return False
            remain = t_end - time.time()
            if remain > 0 and abs(remain - round(remain)) < 0.06:
                self._log("warn", f"  arming in {round(remain)}...")

        self.mission_start = time.time()
        return True

    def search(self, seen, what):
        """Yaw scan in place until `seen()` returns true. No forward thrust."""
        self.enter(f"SEARCH_{what.upper()}")
        end = time.time() + SEARCH_TIMEOUT
        direction = 1.0
        flip = time.time() + 8.0
        while time.time() < end:
            if seen():
                self._log("info", f"{what} reacquired.")
                return
            if time.time() > flip:
                direction *= -1.0
                flip = time.time() + 16.0
            self.tick(0.0, SCAN_YAW * direction)
        raise Abort(f"{what} not found during search")

    def seek_altitude(self):
        """The one phase in this file that touches z. Drives toward
        self.target_altitude_m above the pool floor using /nucleus_node/
        altimeter_packets, then hands neutral z straight back to ALT_HOLD --
        every other phase holds z at Z_NEUTRAL by design.

        self.altitude_sign is UNVERIFIED (see module docstring): watch the
        altimeter log line for the first few seconds with the vehicle well
        clear of the floor. If altimeter_distance moves the WRONG way
        (shrinking when it should grow, or vice versa), pass
        -p altitude_sign:=-1.0 and rerun. Two backstops guard the floor
        regardless of whether that's been verified yet:

          - self.altitude_min_safe_m aborts immediately on the raw reading,
            checked every tick, independent of altitude_sign or the gains --
            the one check that still works even if the sign is backwards.
          - the diverge check below aborts if the altitude error is
            demonstrably getting WORSE (not better) over one
            ALTITUDE_STALL_CHECK_S window while z is meaningfully off
            neutral -- the signature of a backwards sign -- rather than
            grinding on it for the full ALTITUDE_TIMEOUT.

        x (forward thrust) is pinned to 0.0 for the entire phase -- it never
        drives. r (yaw) DOES stay active, holding gate_heading throughout,
        so a disturbance can't spin the vehicle off heading during the up
        to ALTITUDE_TIMEOUT seconds this phase may take.

        A stale altimeter reading does NOT abort the run: it holds neutral
        z (never drives on a number that might be old) and waits for a
        fresh one. Only ALTITUDE_TIMEOUT overall, or one of the two
        backstops above, can end this phase early.

        Whether ALTITUDE_Z_MAX/ALTITUDE_KP actually produce enough thrust to
        move the vehicle is its own unverified thing -- this tracks the
        altimeter's real rate of change against a reference point every
        ALTITUDE_STALL_CHECK_S seconds, and if it hasn't moved at least
        ALTITUDE_STALL_MIN_MOVE_M despite commanding non-neutral z, logs a
        clear warning instead of silently grinding away with no effect.
        """
        self.enter("SEEK_ALTITUDE")
        self._log("info", "going down to depth")
        if not self.spin_until(lambda: self.altimeter_distance is not None,
                               10.0, "altimeter"):
            raise Abort("no altimeter data -- cannot seek altitude")

        end = time.time() + ALTITUDE_TIMEOUT
        stall_ref_alt, stall_ref_error, stall_ref_time = None, None, None
        while True:
            if time.time() > end:
                raise Abort(f"could not reach {self.target_altitude_m:.2f}m altitude in "
                            f"{ALTITUDE_TIMEOUT:.0f}s")

            alt = self.altimeter_distance
            age = time.time() - self.altimeter_stamp
            if age > ALTIMETER_TIMEOUT:
                self.log_every("seek_altitude_stale", 1.0, lambda: (
                    f"  altimeter stale ({age:.2f}s old, last reading {alt:.2f}m) "
                    f"-- holding, waiting for a fresh one"))
                self.tick(0.0, self.yaw_to(self.gate_heading), z=Z_NEUTRAL)   # never act on a possibly-stale number
                continue

            if alt < self.altitude_min_safe_m:
                raise Abort(f"altimeter {alt:.2f}m < {self.altitude_min_safe_m}m safety floor")

            error = self.target_altitude_m - alt   # +ve: too close to the floor, need more room
            if abs(error) <= ALTITUDE_TOLERANCE_M:
                self._log("info",
                    f"altitude {alt:.2f}m, target {self.target_altitude_m:.2f}m "
                    f"(within {ALTITUDE_TOLERANCE_M:.2f}m). Switching back to ALT_HOLD.")
                self.publish(0.0, self.yaw_to(self.gate_heading), z=Z_NEUTRAL)
                if not self.set_mode("ALT_HOLD"):
                    # STABILIZE does not hold depth -- continuing the rest of
                    # the run believing z=neutral means "depth held" would be
                    # false. Treat this as seriously as any other mode failure.
                    raise Abort("could not switch back to ALT_HOLD after SEEK_ALTITUDE")
                return

            z = clamp(Z_NEUTRAL + self.altitude_sign * ALTITUDE_KP * error,
                     Z_NEUTRAL - ALTITUDE_Z_MAX, Z_NEUTRAL + ALTITUDE_Z_MAX)

            now = time.time()
            if stall_ref_alt is None:
                stall_ref_alt, stall_ref_error, stall_ref_time = alt, error, now
            moved = abs(alt - stall_ref_alt)
            elapsed = now - stall_ref_time
            rate = f"{(alt - stall_ref_alt) / elapsed * 1000:+.0f}mm/s" if elapsed > 0.5 else "measuring..."
            if elapsed > ALTITUDE_STALL_CHECK_S:
                commanding = abs(z - Z_NEUTRAL) > (ALTITUDE_Z_MAX * 0.2)
                diverged = abs(error) - abs(stall_ref_error)
                if commanding and diverged > ALTITUDE_DIVERGE_M:
                    raise Abort(
                        f"altitude error GREW from {stall_ref_error:+.2f}m to {error:+.2f}m "
                        f"over {elapsed:.0f}s while commanding z={z:.0f} -- altitude_sign is "
                        f"almost certainly backwards. Rerun with -p altitude_sign:="
                        f"{-self.altitude_sign:.1f}.")
                if moved < ALTITUDE_STALL_MIN_MOVE_M:
                    self._log("warn",
                        f"commanding z={z:.0f} (delta {z - Z_NEUTRAL:+.0f}) for "
                        f"{elapsed:.0f}s but altitude only moved {moved * 1000:.0f}mm "
                        f"-- not enough thrust? altitude_sign backwards? Consider "
                        f"raising ALTITUDE_KP/ALTITUDE_Z_MAX.")
                stall_ref_alt, stall_ref_error, stall_ref_time = alt, error, now   # fresh window

            # Deliberately NOT labelled ascend/descend: which way z actually
            # moves the vehicle is exactly the unverified thing being
            # watched here. Read the trend of `altimeter` itself against
            # the sign of `z - neutral` to find out, don't trust a label.
            self.log_every("seek_altitude", 1.0, lambda: (
                f"  altimeter {alt:.2f}m  target {self.target_altitude_m:.2f}m  "
                f"error {error:+.2f}m  rate {rate}  age {age:.2f}s  "
                f"z {z:.0f} (neutral {Z_NEUTRAL:.0f}, delta {z - Z_NEUTRAL:+.0f})"))
            self.tick(0.0, self.yaw_to(self.gate_heading), z=z)

    def _prepare_hard_code_descent(self):
        """HARD_CODE mode: turn 'descend N ft' into the absolute target
        altitude that seek_altitude() already knows how to reach, so the
        whole descent -- safety floor, backwards-sign backstops,
        STABILIZE->ALT_HOLD handoff -- is the exact same code the vision
        path uses. Waits for one altimeter reading to learn the launch
        altitude, subtracts the requested descent, and clamps so the target
        never sits below the safety floor (better to stop short than to
        drive at the bottom)."""
        if not self.spin_until(lambda: self.altimeter_distance is not None,
                               10.0, "altimeter"):
            raise Abort("no altimeter data -- cannot descend")
        start_alt = self.altimeter_distance
        target = start_alt - self.hard_down_m
        floor = self.altitude_min_safe_m + ALTITUDE_TOLERANCE_M
        if target < floor:
            self._log("warn",
                f"HARD_CODE: descending {self.hard_down_m:.2f}m from launch altitude "
                f"{start_alt:.2f}m would breach the {self.altitude_min_safe_m:.2f}m safety "
                f"floor -- stopping at {floor:.2f}m above the floor instead "
                f"(only {start_alt - floor:.2f}m of descent).")
            target = floor
        self.target_altitude_m = target
        self._log("info",
            f"HARD_CODE descend: launch altitude {start_alt:.2f}m -> target "
            f"{target:.2f}m above floor (descend {start_alt - target:.2f}m).")

    def hard_code_forward(self):
        """HARD_CODE mode's forward phase: drive hard_code_forward_distance
        on the captured heading using the SAME dead-reckoned INS projection
        (along_cross) blind_push and the vision overshoot use, then stop.
        No vision, no range -- purely 'go this many metres forward'."""
        self.enter("HARD_CODE_FORWARD")
        self._log("info",
            f"moving forward {self.hard_forward_m:.2f}m on heading "
            f"{self.gate_heading:.1f}")
        a0, _ = self.along_cross()
        while True:
            along, _ = self.along_cross()
            travelled = along - a0
            if travelled >= self.hard_forward_m:
                self._log("info",
                    f"HARD_CODE: reached {travelled:.2f}m forward. Stopping.")
                return
            self.log_every("hard_code_forward", 2.0, lambda: (
                f"  forward {along - a0:+.2f}m / {self.hard_forward_m:.2f}m  "
                f"hdg {self.heading:.1f}"))
            self.tick(self.cruise_v, self.yaw_to(self.gate_heading), z=Z_NEUTRAL)

    def settle(self):
        """Hold heading, no forward thrust. There is no depth logic here:
        ALT_HOLD regulates depth on the Cube's own barometer at whatever
        depth we launched at, and z stays neutral for the whole run."""
        self.enter("SETTLE")
        for _ in range(int(SETTLE_SECONDS / DT)):
            self.tick(0.0, self.yaw_to(self.gate_heading))

    def blind_push(self, hdg, metres):
        """Drive `metres` on dead reckoning alone. Only for a short, known
        displacement -- never a guessed one."""
        a0, _ = self.along_cross()
        while True:
            along, _ = self.along_cross()
            if along - a0 >= metres:
                return
            self.tick(self.cruise_v, self.yaw_to(hdg), z=Z_NEUTRAL)

    def drive_to_gate(self):
        """Look for the gate, then move toward it based on the tracked
        state. Gate visible -> drive at it, steering on its live bearing.

        Never seen it at all yet -> stop and actively search (spin in
        place), aborting if that search times out. Seen it before and lost
        it now -> keep pushing forward on the last known heading for up to
        GATE_LOST_STOP seconds (a wave, a bubble, a bad frame, the vehicle
        now being too close to keep it in frame); past that, stop driving
        forward and just hold heading until it reappears (or the mission
        clock / an abort ends things)."""
        self.enter("DRIVE_TO_GATE")
        self._log("info", "moving forward")
        ever_seen = False
        lost_since = None

        while True:
            along, _ = self.along_cross()
            if along > 15.0:
                raise Abort(f"drove {along:.1f}m without reaching the gate")

            b = self.gate_bearing()
            rng, ok = self.gate_range()

            if b is None:
                if not ever_seen:
                    self.search(lambda: self.gate_bearing() is not None, "gate")
                    continue
                if lost_since is None:
                    lost_since = time.time()
                lost_for = time.time() - lost_since
                if lost_for > GATE_LOST_STOP:
                    state = f"stopped (lost {lost_for:.1f}s)"
                    self.tick(0.0, self.yaw_to(self.gate_heading))
                else:
                    state = f"coasting (lost {lost_for:.1f}s)"
                    self.tick(self.cruise_v, self.yaw_to(self.gate_heading), z=Z_NEUTRAL)
            else:
                ever_seen = True
                lost_since = None
                state = "driving"
                sp = (self.heading + b) % 360.0   # centring > heading here
                self.tick(self.cruise_v, self.yaw_to(sp), z=Z_NEUTRAL)
                if ok and rng < GATE_PASS_RANGE:
                    break

            self.log_every("drive_to_gate", 2.0, lambda: (
                f"  [{state}]  gate b {b if b is None else round(b,1)}  "
                f"r {rng if ok else '--'}  conf {self.gate_conf():.2f}  "
                f"along {along:+.2f}  hdg {self.heading:.1f}"))

        self._log("info", f"Gate at {rng:.2f}m. Pushing through.")
        # Our heading right now IS the true gate normal -- better than the
        # heading captured on the surface.
        self.gate_heading = self.heading
        self.blind_push(self.gate_heading, GATE_OVERSHOOT)
        self._log("info", "At the gate.")

    # ---------------- failure ----------------

    def abort(self, why):
        self._log("error", f"ABORT in {self.phase}: {why}")
        for _ in range(10):
            self.neutral()
            time.sleep(DT)
        self.arm(False)
        return 1

    def safe_shutdown(self):
        """Best effort, NOT a safety system -- the hardware kill switch is.
        ArduSub holds the last MANUAL_CONTROL it received, so nothing here
        can help if the process itself is wedged. Set FS_PILOT_INPUT=2 and
        FS_PILOT_TIMEOUT=1.0 on the FCU; this is a convenience on top."""
        if self.dry_run or not self.armed_by_us:
            return
        self._log("warn", "shutdown while armed - neutral + disarm")

        # Zeros first, directly on the publisher: no spinning, no services,
        # nothing that can block. Publish many times since UDP drops packets.
        for _ in range(20):
            try:
                self.neutral()
            except Exception:  # noqa: BLE001
                break
            time.sleep(0.02)

        for attempt in range(3):
            try:
                if self.arm(False):
                    self._log("warn", "disarmed")
                    return
            except Exception as e:  # noqa: BLE001
                self._log("error", f"disarm attempt {attempt+1}: {e}")
            time.sleep(0.3)

        self._log("error",
            "COULD NOT DISARM. Hit the hardware kill switch. "
            "Then: ros2 service call /mavros/cmd/arming "
            "mavros_msgs/srv/CommandBool \"{value: false}\"")


class Abort(Exception):
    pass


def main():
    rclpy.init()
    node = Qualify()

    # Ctrl+C sets the abort flag rather than unwinding mid-service-call.
    # guard() sees it on the next tick and aborts cleanly through the normal
    # path. A second Ctrl+C falls through to KeyboardInterrupt.
    hits = {"n": 0}

    def on_sigint(_sig, _frm):
        hits["n"] += 1
        node._abort_req = True
        node.get_logger().warn(
            "SIGINT: aborting" if hits["n"] == 1
            else "SIGINT again: forcing shutdown")
        if hits["n"] > 1:
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    rc = 1
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl+C")
        node.safe_shutdown()
    except Exception as e:  # noqa: BLE001
        node.get_logger().fatal(f"unhandled: {e}")
        node.safe_shutdown()
    finally:
        node.close_logs()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
