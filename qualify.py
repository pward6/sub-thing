#!/usr/bin/env python3
"""
qualify.py - Nautilus gate run.

    Look for the gate -> drive to it -> stop -> disarm

NO DEPTH LOGIC. Mode is ALT_HOLD: ArduSub regulates depth on its own
barometer and holds whatever depth the vehicle was launched at. This node
never reads depth and never commands vertical thrust. Launch it at the
depth you want it to run at.

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

    ros2 run nautilus_auto qualify --ros-args -p dry_run:=true
    ros2 run nautilus_auto qualify --ros-args -p fake_gate:=true   # no camera needed

PRE-FLIGHT: confirm in water that velocity_nucleus_x goes nonzero and
fom_ins drops below ~5. On the bench it reads 0.0 / 44.9 and the position
solution is meaningless. Run with require_bottom_lock:=true to hard-gate it.
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
Z_NEUTRAL = 500.0   # ManualControl z: 500 = no vertical demand (ALT_HOLD)
DT = 0.05           # 20 Hz control loop
FAKE_GATE_SECONDS = 10.0   # fake_gate:=true fakes a solid gate for this long, once

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

        d = self.declare_parameter
        d("dry_run", False)
        d("fake_gate", False)               # bench: fake a solid gate, no camera needed
        d("fake_gate_bearing_deg", 0.0)     # bearing to report while faking
        d("fake_gate_range_m", 3.0)         # range to report while faking (range_ok=true)
        d("target_heading", float("nan"))   # NaN -> capture at arm
        d("cruise_speed", 0.35)             # thrust fraction while driving
        d("arm_delay", 5.0)                 # s between gate acquired and arming
        d("mission_timeout", 1000.0)        # s, hard abort -- runaway guard, not a schedule
        d("gate_acquire_timeout", 0.0)      # s to wait on deck. 0 = forever.
        d("skip_gate_wait", False)          # bench only: arm without seeing a gate
        d("require_bottom_lock", False)     # hard-gate on DVL bottom lock at launch
        d("max_fom_ins", 10.0)              # only enforced if require_bottom_lock

        g = lambda n: self.get_parameter(n).value
        self.dry_run = bool(g("dry_run"))
        self.fake_gate = bool(g("fake_gate"))
        self.fake_bearing = float(g("fake_gate_bearing_deg"))
        self.fake_range = float(g("fake_gate_range_m"))
        self.gate_heading = float(g("target_heading"))
        self.cruise_v = float(g("cruise_speed")) * 1000.0
        self.arm_delay = float(g("arm_delay"))
        self.mission_timeout = float(g("mission_timeout"))
        self.acquire_timeout = float(g("gate_acquire_timeout"))
        self.require_lock = bool(g("require_bottom_lock"))
        self.max_fom = float(g("max_fom_ins"))
        self._fake_until = None   # lazily set on first gate check, not at startup

        # INS state
        self.heading = None
        self.pos = None          # (x, y) in INS frame
        self.vel_x = 0.0
        self.fom_ins = 999.0
        self.ins_stamp = 0.0

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

        self.create_subscription(State, "/mavros/state", self._on_state,
                                 self._match_qos("/mavros/state"))
        self.ctrl = self.create_publisher(ManualControl,
                                          "/mavros/manual_control/send", 10)
        self._subscribe_ins(qos)
        self.create_subscription(String, "/nautilus/detections",
                                 self._on_detections, 10)
        self.create_subscription(Empty, "/nautilus/cmd/abort",
                                 self._on_abort, 10)
        self._abort_req = False

        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.state = None
        self.state_stamp = 0.0
        if self.dry_run:
            self.get_logger().warn("DRY RUN: no arm, no thrust published.")
        if self.fake_gate:
            self.get_logger().warn(
                f"FAKE_GATE: gate will be faked CONFIRMED at bearing "
                f"{self.fake_bearing:+.1f} deg, range {self.fake_range:.1f}m, "
                f"for {FAKE_GATE_SECONDS:.0f}s the first time it's checked.")

        self._cmd_log = self._open_cmd_log()

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

    def _log_cmd(self, x, y, z, r):
        self._cmd_log.write(
            f"{time.time():.3f},{self.phase},{x:.1f},{y:.1f},{z:.1f},{r:.1f}\n")

    def close_cmd_log(self):
        try:
            self._cmd_log.close()
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
                self.get_logger().info(
                    f"{topic}: matching pub QoS "
                    f"{q.reliability.name}/{q.durability.name}")
                return QoSProfile(
                    reliability=q.reliability,
                    durability=q.durability,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10)
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().warn(f"{topic}: no publisher, using default QoS")
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
            self.get_logger().fatal(f"{INS_TOPIC} absent. nautilus_up.sh running?")
            raise SystemExit(1)
        self.get_logger().info(f"INS type: {mt}")
        self.create_subscription(get_message(mt), INS_TOPIC, self._on_ins, qos)

    # ---------------- callbacks ----------------

    def _on_state(self, msg):
        self.state = msg
        self.state_stamp = time.time()

    def _on_abort(self, _msg):
        """Convenience kill, NOT a safety system -- the hardware kill switch
        is. ros2 topic pub --once /nautilus/cmd/abort std_msgs/msg/Empty {}"""
        self.get_logger().warn("ABORT REQUESTED on /nautilus/cmd/abort")
        self._abort_req = True

    def fcu_live(self):
        """connected:true AND recent. /mavros/state is latched, so a stale
        "connected:true" from a dead MAVROS must not be trusted."""
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
        or whatever's actually fresh from pipe_detector.py."""
        if self.fake_gate:
            if self._fake_until is None:
                self._fake_until = time.time() + FAKE_GATE_SECONDS
            if time.time() < self._fake_until:
                return {"bearing_deg": self.fake_bearing, "range_m": self.fake_range,
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
            self.get_logger().error(f"{what}: service unavailable")
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def arm(self, value):
        if self.dry_run:
            self.get_logger().info(f"[dry_run] arm({value})")
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
            self.get_logger().info(f"[dry_run] set_mode({mode})")
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
        self.get_logger().error(f"timeout waiting for {what}")
        return False

    def enter(self, phase):
        self.phase = phase
        self.reset_pi()
        self.get_logger().info(f"=== {phase} ===")

    def log_every(self, key, interval, msg):
        """Throttled info log, shared by every phase's progress line."""
        now = time.time()
        if now - self._log_times.get(key, 0.0) > interval:
            self._log_times[key] = now
            self.get_logger().info(msg())

    # ---------------- mission ----------------

    def run(self):
        self.enter("WAIT_FCU")
        if not self.spin_until(self.fcu_live, 30.0, "a fresh connected:true"):
            if self.state is None:
                self.get_logger().error(
                    "no /mavros/state at all. Is MAVROS running? "
                    "pgrep -af mavros_node")
            elif not self.state.connected:
                self.get_logger().error(
                    "MAVROS up, FCU silent. Cube powered? TELEM2 harness?")
            else:
                self.get_logger().error(
                    "connected:true but STALE - that is a latched message "
                    "from a dead MAVROS. Check: ros2 topic hz /mavros/state")
            return 1

        self.enter("WAIT_INS")
        if not self.spin_until(lambda: self.pos is not None, 20.0, "INS"):
            return 1

        self.get_logger().info(
            f"hdg {self.heading:.1f}  fom_ins {self.fom_ins:.1f}  "
            f"vel_x {self.vel_x:.3f}")
        if self.fom_ins > self.max_fom:
            msg = (f"fom_ins={self.fom_ins:.1f} > {self.max_fom} - no bottom "
                   f"lock. Dead reckoning will drift.")
            if self.require_lock:
                self.get_logger().fatal(msg)
                return 1
            self.get_logger().warn(msg + " Proceeding anyway.")

        if not self.wait_for_gate():
            return 1

        if math.isnan(self.gate_heading):
            self.gate_heading = self.heading
            self.get_logger().info(f"Gate heading CAPTURED: {self.gate_heading:.1f}")
        self.origin = self.pos

        self.enter("ARM")
        self.neutral()
        if not self.arm(True):
            self.get_logger().fatal("arm rejected (SYSID_MYGCS=1?)")
            return 1
        if not self.set_mode("ALT_HOLD"):
            self.get_logger().fatal("mode rejected")
            self.arm(False)
            return 1

        try:
            self.settle()
            self.drive_to_gate()
        except Abort as e:
            return self.abort(str(e))

        self.enter("DISARM")
        for _ in range(20):
            self.neutral()
            rclpy.spin_once(self, timeout_sec=DT)
        self.arm(False)
        self.get_logger().info("Run complete.")
        return 0

    def wait_for_gate(self):
        """Hold on the surface, disarmed, until pipe_detector.py reports a
        confirmed gate (or fake_gate fakes one). The run does not begin on
        an enter keypress -- it begins when the vehicle can see what it's
        aiming at. skip_gate_wait bypasses this on the bench."""
        self.enter("WAIT_GATE")
        if bool(self.get_parameter("skip_gate_wait").value):
            self.get_logger().warn("skip_gate_wait: arming without a gate.")
            self.mission_start = time.time()
            return True

        forever = self.acquire_timeout <= 0.0
        deadline = None if forever else time.time() + self.acquire_timeout
        if forever:
            self.get_logger().info(
                "Waiting indefinitely for the gate. Disarmed, no thrust. "
                "Ctrl+C or /nautilus/cmd/abort to stop.")

        while rclpy.ok():
            if self._abort_req:
                self.get_logger().error("Aborted before arming.")
                return False
            if deadline and time.time() > deadline:
                self.get_logger().error("Gate never appeared. Not arming.")
                return False

            rclpy.spin_once(self, timeout_sec=0.05)
            if self.ins_ok() and self.gate_is_solid():
                break

            def _status():
                left = "" if forever else f"  ({deadline - time.time():.0f}s left)"
                b = self.gate_bearing()
                why = "none" if b is None else f"weak (conf {self.gate_conf():.2f})"
                return (f"  waiting for gate [{why}]  hdg {self.heading:.1f}  "
                       f"fom {self.fom_ins:.1f}{left}")
            self.log_every("wait_gate", 5.0, _status)
        else:
            return False

        b = self.gate_bearing()
        rng, ok = self.gate_range()
        self.get_logger().warn(
            f"GATE ACQUIRED: bearing {b:+.1f}, range {rng if ok else 'far'}. "
            f"ARMING IN {self.arm_delay:.0f}s -- HANDS OFF.")

        # Countdown. A confirmed gate is not consent to arm instantly.
        t_end = time.time() + self.arm_delay
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._abort_req:
                self.get_logger().error("Aborted during arm countdown.")
                return False
            remain = t_end - time.time()
            if remain > 0 and abs(remain - round(remain)) < 0.06:
                self.get_logger().warn(f"  arming in {round(remain)}...")

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
                self.get_logger().info(f"{what} reacquired.")
                return
            if time.time() > flip:
                direction *= -1.0
                flip = time.time() + 16.0
            self.tick(0.0, SCAN_YAW * direction)
        raise Abort(f"{what} not found during search")

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
        Gate not seen -> zero forward thrust, spin in place looking for it.
        Losing the gate briefly is normal (a wave, a bubble, a bad frame)
        and just pauses advance; only an extended loss triggers an active
        search, which aborts the run if it times out."""
        self.enter("DRIVE_TO_GATE")
        lost_since = None

        while True:
            along, _ = self.along_cross()
            if along > 15.0:
                raise Abort(f"drove {along:.1f}m without reaching the gate")

            b = self.gate_bearing()
            rng, ok = self.gate_range()

            if b is None:
                if lost_since is None:
                    lost_since = time.time()
                elif time.time() - lost_since > 3.0:
                    self.search(lambda: self.gate_bearing() is not None, "gate")
                    lost_since = None
                    continue
                self.tick(0.0, self.yaw_to(self.gate_heading))
            else:
                lost_since = None
                sp = (self.heading + b) % 360.0   # centring > heading here
                self.tick(self.cruise_v, self.yaw_to(sp), z=Z_NEUTRAL)
                if ok and rng < GATE_PASS_RANGE:
                    break

            self.log_every("drive_to_gate", 2.0, lambda: (
                f"  gate b {b if b is None else round(b,1)}  "
                f"r {rng if ok else '--'}  conf {self.gate_conf():.2f}  "
                f"along {along:+.2f}"))

        self.get_logger().info(f"Gate at {rng:.2f}m. Pushing through.")
        # Our heading right now IS the true gate normal -- better than the
        # heading captured on the surface.
        self.gate_heading = self.heading
        self.blind_push(self.gate_heading, GATE_OVERSHOOT)
        self.get_logger().info("At the gate.")

    # ---------------- failure ----------------

    def abort(self, why):
        self.get_logger().error(f"ABORT in {self.phase}: {why}")
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
        self.get_logger().warn("shutdown while armed - neutral + disarm")

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
                    self.get_logger().warn("disarmed")
                    return
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"disarm attempt {attempt+1}: {e}")
            time.sleep(0.3)

        self.get_logger().error(
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
        node.close_cmd_log()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
