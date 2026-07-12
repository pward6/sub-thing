#!/usr/bin/env python3
"""
qualify.py - Nautilus RoboSub qualifying run.

    Gate -> transit to pole -> 180 u-turn -> return through gate -> disarm

NO DEPTH LOGIC. Mode is ALT_HOLD: ArduSub regulates depth on its own
barometer and holds whatever depth the vehicle was launched at. This node
never reads depth and never commands vertical thrust -- no ceiling guard,
no over-depth abort, no surfacing at the end. Launch it at the depth you
want it to run at.

Vision (pipe_detector.py, /nautilus/detections) owns WHERE the targets are;
no phase terminates on a dead-reckoned distance. It also does its own
confirm-streak state tracking now (see tracking.py) -- a gate is only
"confirmed" there once several consecutive frames agree on bearing, and a
brief dropout doesn't erase it. This node's job is therefore just: trust
"confirmed" at ARM time (the one moment that matters most), and use
whatever's fresh during the approach phases (a weaker reading is fine once
under way -- it's corroborated by everything before it).

The INS owns heading, holds a straight line when a target is out of frame,
and supplies the sanity bounds that abort the run if we drive somewhere
absurd.

    ros2 run nautilus_auto qualify --ros-args -p dry_run:=true
    ros2 run nautilus_auto qualify --ros-args -p vision_gain:=0.0   # INS only

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
TEST_FORWARD_SECONDS = 2.0   # bench smoke test: see gate -> push forward this long -> stop


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Qualify(Node):

    # ---------------- setup ----------------

    def __init__(self):
        super().__init__("qualify")

        d = self.declare_parameter
        d("pole_distance_max", 45.0)  # ABORT bound, not a target
        d("turn_radius", 3.0)         # m, radius of the U-turn about the pole
        d("turn_speed", 0.25)         # thrust fraction during the U-turn
        d("vision_gain", 0.6)         # 0 = ignore camera, 1 = trust it fully
        d("vision_timeout", 1.5)      # s before a detection goes stale
        d("cruise_speed", 0.35)       # thrust fraction during transit
        d("settle_seconds", 3.0)      # s of heading hold before driving
        d("gate_overshoot", 2.5)      # m of blind push once the gate leaves frame
        d("gate_pass_range", 1.6)     # m: closer than this, commit to passing
        d("pole_stop_offset", 1.0)    # m added to turn_radius for U-turn trigger
        d("search_yaw", 200.0)        # yaw demand while scanning
        d("search_timeout", 90.0)
        d("gate_acquire_timeout", 0.0)    # s to wait on deck. 0 = forever.
        d("arm_delay", 5.0)               # s between gate acquired and arming
        d("gate_min_conf", 0.20)          # sanity floor at ARM time only
        d("scan_yaw_slow", 120.0)         # gentle scan while hunting the gate
        d("lost_grace", 8.0)              # s a target may vanish before we search
        d("return_margin", 4.0)       # m past gate on the way back
        d("kp_heading", 6.0)
        d("ki_heading", 0.4)
        d("yaw_limit", 400.0)
        d("i_limit", 200.0)
        d("ins_timeout", 1.0)         # s
        d("state_max_age", 5.0)       # s: reject latched/stale /mavros/state
        # Runaway guard, NOT a schedule. Battery is the real time limit.
        d("mission_timeout", 1000.0)  # s, hard abort
        d("uturn_timeout", 120.0)     # s
        d("uturn_stall_s", 15.0)      # s without heading progress -> abort
        d("require_bottom_lock", False)
        d("max_fom_ins", 10.0)        # only enforced if require_bottom_lock
        d("target_heading", float("nan"))   # NaN -> capture at arm
        d("startup_delay", 0.0)       # s to sit still after launch
        d("skip_gate_wait", False)    # bench only: arm without seeing a gate
        d("dry_run", False)
        d("cmd_log_dir", "~/nautilus_ws/logs")   # every command actually sent, csv
        d("test_forward_only", False)   # bench: after arming, push forward
                                         # TEST_FORWARD_SECONDS then disarm --
                                         # skips the rest of the course

        g = lambda n: self.get_parameter(n).value
        self.pole_d_max = float(g("pole_distance_max"))
        self.turn_r = float(g("turn_radius"))
        self.turn_v = float(g("turn_speed")) * 1000.0
        self.vis_gain = float(g("vision_gain"))
        self.vis_timeout = float(g("vision_timeout"))
        self.cruise_v = float(g("cruise_speed")) * 1000.0
        self.settle_s = float(g("settle_seconds"))
        self.overshoot = float(g("gate_overshoot"))
        self.gate_pass_r = float(g("gate_pass_range"))
        self.pole_stop_off = float(g("pole_stop_offset"))
        self.search_yaw = float(g("search_yaw"))
        self.search_timeout = float(g("search_timeout"))
        self.acquire_timeout = float(g("gate_acquire_timeout"))
        self.arm_delay = float(g("arm_delay"))
        self.gate_min_conf = float(g("gate_min_conf"))
        self.scan_yaw_slow = float(g("scan_yaw_slow"))
        self.lost_grace = float(g("lost_grace"))
        self.return_margin = float(g("return_margin"))
        self.kp = float(g("kp_heading"))
        self.ki = float(g("ki_heading"))
        self.yaw_limit = float(g("yaw_limit"))
        self.i_limit = float(g("i_limit"))
        self.ins_timeout = float(g("ins_timeout"))
        self.state_max_age = float(g("state_max_age"))
        self.mission_timeout = float(g("mission_timeout"))
        self.uturn_timeout = float(g("uturn_timeout"))
        self.uturn_stall = float(g("uturn_stall_s"))
        self.require_lock = bool(g("require_bottom_lock"))
        self.max_fom = float(g("max_fom_ins"))
        self.gate_heading = float(g("target_heading"))
        self.dry_run = bool(g("dry_run"))
        self.startup_delay = float(g("startup_delay"))
        self.test_forward_only = bool(g("test_forward_only"))

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

        # Vision: each is the pipe_detector.py "gate"/"pole" dict plus
        # recv_t, or None. pipe_detector.py already does confirm-streak and
        # hold-through-miss (age_s); recv_t catches the case where
        # pipe_detector itself has died and stopped publishing entirely.
        self._gate = None
        self._pole = None
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

        self._cmd_log = self._open_cmd_log(str(g("cmd_log_dir")))

    def _open_cmd_log(self, log_dir):
        """Every command this node ever sends (or would send, in dry_run)
        goes here as it's published -- x/y/z/r, post-clamp, with the phase
        active at the time. This is the actual record of what moved the
        vehicle, independent of console log level or whether a bag was
        recording /mavros/manual_control/send.
        """
        path = os.path.join(os.path.expanduser(log_dir),
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
        return age <= self.state_max_age

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
        now = time.time()
        if d.get("gate"):
            self._gate = dict(d["gate"], recv_t=now)
        if d.get("pole"):
            self._pole = dict(d["pole"], recv_t=now)

    def _fresh(self, rec):
        """A detection is fresh only if BOTH the message itself just
        arrived (catches pipe_detector.py dying) AND its own age_s was
        small when published (catches "still publishing, but hasn't
        actually seen this in a while" -- pipe_detector.py holds a
        confirmed reading through brief misses, so message arrival alone
        is not enough)."""
        if not rec:
            return None
        total_age = rec["age_s"] + (time.time() - rec["recv_t"])
        return rec if total_age < self.vis_timeout else None

    def gate_bearing(self):
        r = self._fresh(self._gate)
        return r["bearing_deg"] if r else None

    def gate_range(self):
        """(range_m, ok) or (None, False) when no fresh detection."""
        r = self._fresh(self._gate)
        return (r["range_m"], r["range_ok"]) if r else (None, False)

    def gate_conf(self):
        r = self._fresh(self._gate)
        return r["conf"] if r else 0.0

    def gate_is_solid(self):
        """A gate good enough to ARM on. pipe_detector.py already requires
        several consecutive agreeing frames before it marks "confirmed";
        this just adds a confidence floor on top, and is ONLY applied at
        acquisition. Once approaching, a weaker reading is fine -- it's
        corroborated by everything before it."""
        r = self._fresh(self._gate)
        return bool(r and r.get("confirmed") and r["conf"] >= self.gate_min_conf)

    def pole_bearing(self):
        r = self._fresh(self._pole)
        return r["bearing_deg"] if r else None

    def pole_range(self):
        r = self._fresh(self._pole)
        return (r["range_m"], r["range_ok"]) if r else (None, False)

    def visual_setpoint(self, base_hdg, bearing):
        """Blend a camera bearing into a dead-reckoned heading setpoint.
        vision_gain=0 disables it entirely -- correct if the detector is
        misbehaving poolside."""
        if bearing is None or self.vis_gain <= 0.0:
            return base_hdg
        return (self.heading + self.vis_gain * bearing) % 360.0

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
        if (time.time() - self.ins_stamp) > self.ins_timeout:
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
            self.integral = clamp(self.integral + err * dt,
                                  -self.i_limit, self.i_limit)
        return clamp(self.kp * err + self.ki * self.integral,
                     -self.yaw_limit, self.yaw_limit)

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

        if self.startup_delay > 0:
            self.get_logger().info(f"startup_delay {self.startup_delay:.0f}s")
            t = time.time() + self.startup_delay
            while time.time() < t and not self._abort_req:
                rclpy.spin_once(self, timeout_sec=0.1)

        if not self.wait_for_gate():
            return 1

        if math.isnan(self.gate_heading):
            self.gate_heading = self.heading
            self.get_logger().info(f"Gate heading CAPTURED: {self.gate_heading:.1f}")
        self.origin = self.pos
        self.return_heading = (self.gate_heading + 180.0) % 360.0

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
            if self.test_forward_only:
                self.forward_test()
            else:
                self.through_gate()
                self.to_pole()
                self.uturn()
                self.return_to_origin()
        except Abort as e:
            return self.abort(str(e))

        self.enter("DISARM")
        for _ in range(20):
            self.neutral()
            rclpy.spin_once(self, timeout_sec=DT)
        self.arm(False)
        self.get_logger().info("Qualifying run complete.")
        return 0

    def wait_for_gate(self):
        """Hold on the surface, disarmed, until pipe_detector.py reports a
        confirmed gate. The run does not begin on an enter keypress -- it
        begins when the vehicle can see what it's aiming at. skip_gate_wait
        bypasses this on the bench."""
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
        end = time.time() + self.search_timeout
        direction = 1.0
        flip = time.time() + 8.0
        while time.time() < end:
            if seen():
                self.get_logger().info(f"{what} reacquired.")
                return
            if time.time() > flip:
                direction *= -1.0
                flip = time.time() + 16.0
            self.tick(0.0, self.search_yaw * direction)
        raise Abort(f"{what} not found during search")

    def settle(self):
        """Hold heading, no forward thrust. There is no depth logic here:
        ALT_HOLD regulates depth on the Cube's own barometer at whatever
        depth we launched at, and z stays neutral for the whole run."""
        self.enter("SETTLE")
        for _ in range(int(self.settle_s / DT)):
            self.tick(0.0, self.yaw_to(self.gate_heading))

    def forward_test(self):
        """Bench smoke test: wait_for_gate()/ARM already happened the normal
        way (the vehicle only got here because a gate was actually seen and
        confirmed), so this just proves the vehicle DRIVES once that
        happens -- push forward for TEST_FORWARD_SECONDS, then stop. Not a
        mission phase: replaces through_gate()/to_pole()/uturn()/return_to_
        origin() entirely when test_forward_only:=true. Uses tick(), so the
        normal guards (abort request, INS loss, mission timeout) still apply.
        """
        self.enter("FORWARD_TEST")
        end = time.time() + TEST_FORWARD_SECONDS
        while time.time() < end:
            self.tick(self.cruise_v, self.yaw_to(self.gate_heading))
        self.get_logger().info(f"forward test complete ({TEST_FORWARD_SECONDS:.0f}s).")

    def blind_push(self, hdg, metres):
        """Drive `metres` on dead reckoning alone. Only for a short, known
        displacement -- never a guessed one."""
        a0, _ = self.along_cross()
        while True:
            along, _ = self.along_cross()
            if along - a0 >= metres:
                return
            self.tick(self.cruise_v, self.yaw_to(hdg), z=Z_NEUTRAL)

    def through_gate(self):
        """Approach and pass the gate. Gate visible -> drive at it, full
        vision gain. Gate not seen -> zero forward thrust, gentle yaw scan.
        Losing the gate is normal (a wave, a bubble, a bad frame) and just
        pauses advance -- it only ends via passing the gate, driving 15 m
        without passing it, the mission clock, or a guard failure."""
        self.enter("THROUGH_GATE")
        scan_dir = 1.0
        scan_flip = time.time() + 6.0

        while True:
            along, _ = self.along_cross()
            if along > 15.0:
                raise Abort(f"drove {along:.1f}m without passing the gate")

            b = self.gate_bearing()
            rng, ok = self.gate_range()

            if b is None:
                if time.time() > scan_flip:
                    scan_dir *= -1.0
                    scan_flip = time.time() + 12.0
                self.tick(0.0, self.scan_yaw_slow * scan_dir)
            else:
                scan_flip = time.time() + 6.0
                scan_dir = 1.0
                # Centring matters more than heading here: full vision gain.
                sp = (self.heading + b) % 360.0
                self.tick(self.cruise_v, self.yaw_to(sp), z=Z_NEUTRAL)
                if ok and rng < self.gate_pass_r:
                    break

            self.log_every("through_gate", 2.0, lambda: (
                f"  gate b {b if b is None else round(b,1)}  "
                f"r {rng if ok else '--'}  conf {self.gate_conf():.2f}  "
                f"along {along:+.2f}"))

        self.get_logger().info(f"Gate at {rng:.2f}m. Pushing through.")
        # Our heading right now IS the true gate normal -- better than the
        # heading captured on the surface. Everything downstream inherits it.
        self.gate_heading = self.heading
        self.return_heading = (self.gate_heading + 180.0) % 360.0
        self.blind_push(self.gate_heading, self.overshoot)
        self.get_logger().info("Through the gate.")

    def to_pole(self):
        """Far field: steer the pole's bearing, no range needed. Near field:
        stereo range becomes trustworthy, stop at turn radius.
        pole_distance_max is an abort bound, never a target."""
        self.enter("TO_POLE")
        stop_at = self.turn_r + self.pole_stop_off
        lost_since = None

        while True:
            along, cross = self.along_cross()
            if along > self.pole_d_max:
                raise Abort(f"{along:.1f}m out, past the sanity bound")

            b = self.pole_bearing()
            rng, ok = self.pole_range()

            if b is None:
                if lost_since is None:
                    lost_since = time.time()
                elif time.time() - lost_since > self.lost_grace * 1.5:
                    self.search(lambda: self.pole_bearing() is not None, "pole")
                    lost_since = None
                    continue
                # Coast straight on the gate axis while it is out of frame.
                self.tick(self.cruise_v, self.yaw_to(self.gate_heading))
            else:
                lost_since = None
                sp = self.visual_setpoint(self.gate_heading, b)
                self.tick(self.cruise_v, self.yaw_to(sp))
                if ok and rng <= stop_at:
                    self.get_logger().info(f"Pole at {rng:.2f}m. Turning.")
                    return

            self.log_every("to_pole", 2.0, lambda: (
                f"  pole b {b if b is None else round(b,1)}  "
                f"r {rng if ok else '--'}  along {along:+6.2f}  "
                f"cross {cross:+5.2f}  fom {self.fom_ins:.1f}"))

    def uturn(self):
        """180 deg arc around the pole. Constant forward thrust plus constant
        yaw traces a circle of radius v/omega, stopping at 180 deg of
        accumulated heading change. Radius is NOT measured -- yaw_cmd is a
        starting guess; tune it in a pool by running this phase alone."""
        self.enter("UTURN")
        yaw_cmd = clamp(self.turn_v * 0.9 / max(self.turn_r, 1.0), 80.0, 350.0)
        self.get_logger().info(
            f"radius ~{self.turn_r:.1f}m, yaw_cmd {yaw_cmd:.0f}, 180 deg")

        turned = 0.0
        prev = self.heading
        hard = time.time() + self.uturn_timeout
        best = 0.0
        last_progress = time.time()

        while turned < 178.0:
            if time.time() > hard:
                raise Abort(f"u-turn exceeded {self.uturn_timeout:.0f}s "
                            f"({turned:.0f} of 180 deg)")
            if turned > best + 5.0:
                best = turned
                last_progress = time.time()
            elif time.time() - last_progress > self.uturn_stall:
                raise Abort(f"u-turn stalled at {turned:.0f} deg. "
                            f"Yaw authority? Thruster failure?")

            self.tick(self.turn_v, yaw_cmd)
            turned += abs(wrap180(self.heading - prev))
            prev = self.heading
            self.log_every("uturn", 2.0, lambda: f"  turned {turned:5.1f} deg")
        self.get_logger().info(f"U-turn complete, hdg {self.heading:.1f}")

    def return_to_origin(self):
        """Steer to the live bearing toward the launch point (so the u-turn's
        lateral offset self-corrects), then continue return_margin metres
        past the gate."""
        self.enter("RETURN")
        while True:
            along, cross = self.along_cross()
            if along <= -self.return_margin:
                break

            dx = self.origin[0] - self.pos[0]
            dy = self.origin[1] - self.pos[1]
            dist = math.hypot(dx, dy)

            if dist > 1.0:
                sp = math.degrees(math.atan2(dy, dx)) % 360.0
            else:
                # On top of the origin: just push through on the reciprocal.
                sp = self.return_heading

            # Once close to the gate, let vision centre us in it.
            gb = self.gate_bearing()
            if along < 6.0 and gb is not None:
                sp = (self.heading + gb) % 360.0

            self.tick(self.cruise_v, self.yaw_to(sp), z=Z_NEUTRAL)
            self.log_every("return", 2.0, lambda: (
                f"  along {along:+6.2f}  cross {cross:+5.2f}  "
                f"dist {dist:5.2f}  sp {sp:6.1f}  hdg {self.heading:6.1f}"))
        self.get_logger().info("Back through the gate.")

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
