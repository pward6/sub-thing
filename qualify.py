#!/usr/bin/env python3
"""
qualify.py - Nautilus RoboSub qualifying run.

    Gate -> transit to pole -> 180 u-turn -> return through gate -> disarm

NO DEPTH LOGIC
    This node never reads depth and never commands vertical thrust. Mode is
    ALT_HOLD, so ArduSub regulates depth on its own barometer and holds
    whatever depth the vehicle was launched at.

    Removed with it: the crossbar ceiling guard, the over-depth abort, the
    end-of-run surfacing, and the submerged interlock. Launch the vehicle at
    the depth you want it to run at. If it clips the crossbar, it clips it.

DIVISION OF LABOUR
    Vision owns WHERE the targets are. Neither the gate nor the pole sits at
    a known distance, so no phase terminates on a dead-reckoned number.
    The gate leg ends when stereo says the gate is close. The pole leg ends
    when stereo says the pole is within turn radius.

    The INS owns heading, holds a straight line when a target is out of frame,
    flies the return leg back to the launch point, and supplies the sanity
    bounds that abort the run if we drive somewhere absurd.

    Requires pipe_detector.py publishing /nautilus/detections.

    ros2 run nautilus_auto qualify --ros-args -p dry_run:=true
    ros2 run nautilus_auto qualify --ros-args -p vision_gain:=0.0   # INS only

CRITICAL PRE-FLIGHT: confirm in water that velocity_nucleus_x goes nonzero
and fom_ins drops below ~5. On the bench it reads 0.0 / 44.9 and the position
solution is meaningless. Run with require_bottom_lock:=true to hard-gate on it.
"""

import json
import math
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

Z_NEUTRAL = 500.0   # ManualControl z: 500 = no vertical demand.
                    # ALT_HOLD holds launch depth on the Cube's baro.

DT = 0.05           # 20 Hz control loop


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
        d("gate_confirm_frames", 6)       # consecutive detections before arming
        d("gate_min_conf", 0.20)          # reject weak detections
        d("gate_acquire_needs_crossbar", True)   # only a crossbar match arms
        d("gate_bearing_jitter", 12.0)    # deg: bearing must be steady to confirm
        d("scan_yaw_slow", 120.0)         # gentle scan while hunting the gate
        d("lost_grace", 8.0)              # s a target may vanish before we search
        d("return_margin", 4.0)       # m past gate on the way back
        d("kp_heading", 6.0)
        d("ki_heading", 0.4)
        d("yaw_limit", 400.0)
        d("i_limit", 200.0)
        d("ins_timeout", 1.0)         # s
        d("state_max_age", 5.0)       # s: reject latched/stale /mavros/state
        # Runaway guard, NOT a schedule. It is the only bound on the loops
        # that scan or coast indefinitely. Size it as "the run has clearly
        # failed", not "the run should be done". Battery is the real limit.
        d("mission_timeout", 1000.0)  # s, hard abort
        d("uturn_timeout", 120.0)     # s
        d("uturn_stall_s", 15.0)      # s without heading progress -> abort
        d("require_bottom_lock", False)
        d("max_fom_ins", 10.0)        # only enforced if require_bottom_lock
        d("target_heading", float("nan"))   # NaN -> capture at arm
        d("startup_delay", 0.0)       # s to sit still after launch
        d("skip_gate_wait", False)    # bench only: arm without seeing a gate
        d("dry_run", False)

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
        self.confirm_frames = int(g("gate_confirm_frames"))
        self.gate_min_conf = float(g("gate_min_conf"))
        self.need_crossbar = bool(g("gate_acquire_needs_crossbar"))
        self.bearing_jitter = float(g("gate_bearing_jitter"))
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

        # Vision. Populated by /nautilus/detections; None when stale.
        self._gate = None            # (bearing_deg, width_px, conf, stamp)
        self._pole = None

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

    def _match_qos(self, topic, timeout=8.0):
        """Mirror the publisher's QoS.

        MAVROS publishes /mavros/state transient-local, and may use
        best-effort. A subscriber whose profile does not match receives
        nothing, silently, forever -- which reads as "FCU never connected".
        Rosbag auto-detects; we must too.
        """
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
        """Software kill. Publish once to stop the run from any terminal:

            ros2 topic pub --once /nautilus/cmd/abort std_msgs/msg/Empty {}

        This is a convenience, NOT a safety system. The hardware kill switch
        is the safety system. Software cannot be trusted to stop software.
        """
        self.get_logger().warn("ABORT REQUESTED on /nautilus/cmd/abort")
        self._abort_req = True

    def fcu_live(self):
        """connected:true AND recent.

        /mavros/state is latched. A subscriber joining after MAVROS dies
        receives the last sample it ever sent -- possibly hours old, possibly
        saying connected:true. Arming on that would be very bad. Trust the
        header stamp, not the flag.
        """
        if self.state is None or not self.state.connected:
            return False
        hdr = self.state.header.stamp
        age = self.get_clock().now().nanoseconds * 1e-9 - (
            hdr.sec + hdr.nanosec * 1e-9)
        if age > self.state_max_age:
            return False
        return True

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
        g = d.get("gate")
        p = d.get("pole")
        if g:
            self._gate = (g["bearing"], g.get("range", 0.0),
                          bool(g.get("range_ok")), g.get("crossbar_y"), now,
                          float(g.get("conf", 0.0)), g.get("src", "?"))
        if p:
            self._pole = (p["bearing"], p.get("range", 0.0),
                          bool(p.get("range_ok")), now)

    def _fresh(self, rec, stamp_idx):
        return rec if rec and (time.time() - rec[stamp_idx]) < self.vis_timeout else None

    def gate_bearing(self):
        r = self._fresh(self._gate, 4)
        return r[0] if r else None

    def gate_range(self):
        """(range_m, ok) or (None, False) when no fresh detection."""
        r = self._fresh(self._gate, 4)
        return (r[1], r[2]) if r else (None, False)

    def crossbar_y(self):
        r = self._fresh(self._gate, 4)
        return r[3] if r else None

    def gate_quality(self):
        """(conf, src) of the freshest gate, or (0.0, None)."""
        r = self._fresh(self._gate, 4)
        return (r[5], r[6]) if r else (0.0, None)

    def gate_is_solid(self):
        """A gate good enough to arm on.

        A desk edge and a door frame will produce two verticals. Requiring a
        matched crossbar, decent confidence, and a valid stereo range makes
        the false positive much harder. This gate is only applied at
        ACQUISITION -- once we are approaching, a weaker detection is fine
        because it is corroborated by everything before it.
        """
        b = self.gate_bearing()
        if b is None:
            return False
        conf, src = self.gate_quality()
        if conf < self.gate_min_conf:
            return False
        if self.need_crossbar and src != "crossbar":
            return False
        _, ok = self.gate_range()
        return ok

    def pole_bearing(self):
        r = self._fresh(self._pole, 3)
        return r[0] if r else None

    def pole_range(self):
        r = self._fresh(self._pole, 3)
        return (r[1], r[2]) if r else (None, False)

    def visual_setpoint(self, base_hdg, bearing):
        """Blend a camera bearing into a dead-reckoned heading setpoint.

        Vision is a nudge, not an override. vision_gain=0 disables it entirely,
        which is the correct setting if the detector is misbehaving poolside.
        """
        if bearing is None or self.vis_gain <= 0.0:
            return base_hdg
        return (self.heading + self.vis_gain * bearing) % 360.0

    # ---------------- geometry ----------------

    def along_cross(self):
        """Position projected onto the gate axis.

        Returns (along, cross) in meters. `along` is distance past the gate
        along the run heading; `cross` is lateral offset, +right.
        """
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
        """Hold on the surface, disarmed, until the gate is in view.

        The run does not begin when someone presses enter on the dock. It
        begins when the vehicle can see what it is aiming at. Launch the
        script, lower the vehicle, walk away. Nothing is armed and no thrust
        is published until gate_confirm_frames consecutive detections land.

        skip_gate_wait:=true bypasses this for bench work.
        """
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

        streak = 0
        first_b = 0.0
        last_log = 0.0
        while rclpy.ok():
            if self._abort_req:
                self.get_logger().error("Aborted before arming.")
                return False
            if deadline and time.time() > deadline:
                self.get_logger().error("Gate never appeared. Not arming.")
                return False

            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.ins_ok():
                streak = 0
                continue

            if self.gate_is_solid():
                b = self.gate_bearing()
                if streak and abs(wrap180(b - first_b)) > self.bearing_jitter:
                    streak = 0          # bearing jumped: not the same object
                if streak == 0:
                    first_b = b
                streak += 1
                if streak >= self.confirm_frames:
                    break
            else:
                streak = 0

            if time.time() - last_log > 5.0:
                last_log = time.time()
                left = "" if forever else f"  ({deadline - time.time():.0f}s left)"
                conf, src = self.gate_quality()
                why = "none" if self.gate_bearing() is None else \
                      f"weak (conf {conf:.2f}, src {src})"
                self.get_logger().info(
                    f"  waiting for gate [{why}]  hdg {self.heading:.1f}  "
                    f"fom {self.fom_ins:.1f}{left}")
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
        """Hold heading, no forward thrust, let ALT_HOLD take the depth.

        There is no depth logic in this node. ArduSub's ALT_HOLD regulates
        depth on the Cube's own barometer and holds whatever depth the vehicle
        was launched at. We never command vertical thrust: z stays neutral.

        Consequences of removing depth: no ceiling guard under the crossbar,
        no over-depth abort, no surfacing at the end of the run. The vehicle
        runs at launch depth and disarms where it finishes.
        """
        self.enter("SETTLE")
        for _ in range(int(self.settle_s / DT)):
            self.tick(0.0, self.yaw_to(self.gate_heading))

    def blind_push(self, hdg, metres, phase):
        """Drive `metres` on dead reckoning alone. Used only where DR is
        measuring a short displacement from a known point, never a guessed one."""
        a0, _ = self.along_cross()
        while True:
            along, _ = self.along_cross()
            if along - a0 >= metres:
                return
            self.tick(self.cruise_v, self.yaw_to(hdg), z=Z_NEUTRAL)

    def through_gate(self):
        """Approach and pass the gate. Never move forward without seeing it.

        Two behaviours, one loop:
            gate visible  -> drive at it, full vision gain, ceiling guard on
            gate not seen -> zero forward thrust, gentle yaw scan, wait

        Losing the gate is a NORMAL condition, not an abort. It happens every
        time a wave, a bubble, or a bad frame drops a detection. The vehicle
        simply stops advancing until it can see again. The only things that
        end this phase are: passing the gate, driving 15 m without passing
        it, the mission clock, or a guard failure.

        Nothing here depends on knowing how far the gate is.
        """
        self.enter("THROUGH_GATE")
        last_log = 0.0
        scan_dir = 1.0
        scan_flip = time.time() + 6.0

        while True:
            along, _ = self.along_cross()
            if along > 15.0:
                raise Abort(f"drove {along:.1f}m without passing the gate")

            b = self.gate_bearing()
            rng, ok = self.gate_range()

            if b is None:
                # Hold station-ish and scan. No forward component, ever.
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

            if time.time() - last_log > 2.0:
                last_log = time.time()
                conf, src = self.gate_quality()
                self.get_logger().info(
                    f"  gate b {b if b is None else round(b,1)}  "
                    f"r {rng if ok else '--'}  src {src}  conf {conf:.2f}  "
                    f"bar {self.crossbar_y()}  along {along:+.2f}")

        self.get_logger().info(f"Gate at {rng:.2f}m. Pushing through.")
        # Our heading right now IS the true gate normal. Better than the
        # heading we captured on the surface. Everything downstream inherits it.
        self.gate_heading = self.heading
        self.return_heading = (self.gate_heading + 180.0) % 360.0
        self.blind_push(self.gate_heading, self.overshoot, "GATE_PUSH")
        self.get_logger().info("Through the gate.")

    def to_pole(self):
        """Far field: steer the pole's bearing, no range needed.
        Near field: stereo range becomes trustworthy, stop at turn radius.

        pole_distance_max is an abort bound. It is never a target.
        """
        self.enter("TO_POLE")
        stop_at = self.turn_r + self.pole_stop_off
        lost_since = None
        last_log = 0.0

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

            if time.time() - last_log > 2.0:
                last_log = time.time()
                self.get_logger().info(
                    f"  pole b {b if b is None else round(b,1)}  "
                    f"r {rng if ok else '--'}  along {along:+6.2f}  "
                    f"cross {cross:+5.2f}  fom {self.fom_ins:.1f}")

    def uturn(self):
        """180 deg arc around the pole.

        Constant forward thrust plus constant yaw traces a circle of radius
        v/omega. We stop at 180 deg of accumulated heading change, which puts
        us on the far side of the pole heading back, laterally offset by
        about 2 * turn_radius. The return leg absorbs that offset by steering
        to the origin rather than holding a reciprocal heading.

        Radius is NOT measured. yaw_cmd below is a starting guess -- set it in
        a pool by running this phase alone and watching the arc it draws.
        """
        self.enter("UTURN")
        yaw_cmd = clamp(self.turn_v * 0.9 / max(self.turn_r, 1.0), 80.0, 350.0)
        self.get_logger().info(
            f"radius ~{self.turn_r:.1f}m, yaw_cmd {yaw_cmd:.0f}, 180 deg")

        turned = 0.0
        prev = self.heading
        last_log = 0.0
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
            if time.time() - last_log > 2.0:
                last_log = time.time()
                self.get_logger().info(f"  turned {turned:5.1f} deg")
        self.get_logger().info(f"U-turn complete, hdg {self.heading:.1f}")

    def return_to_origin(self):
        """Steer to the launch point, then continue past the gate.

        Setpoint is the live bearing from current position to the origin, so
        the lateral offset from the u-turn corrects itself. Terminates once we
        are return_margin metres past the gate on the near side.
        """
        self.enter("RETURN")
        last_log = 0.0
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
            if time.time() - last_log > 2.0:
                last_log = time.time()
                self.get_logger().info(
                    f"  along {along:+6.2f}  cross {cross:+5.2f}  "
                    f"dist {dist:5.2f}  sp {sp:6.1f}  hdg {self.heading:6.1f}")
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
        """Best effort. NOT a safety system.

        ArduSub holds the last MANUAL_CONTROL it received -- it thinks a pilot
        is holding the stick. Nothing here can help if the process is wedged,
        DDS has stalled, or the Jetson browned out. The authoritative deadman
        is in the FCU:

            FS_PILOT_INPUT   = 2      (disarm on pilot input loss)
            FS_PILOT_TIMEOUT = 1.0    (seconds)

        Set those. Then this function is a convenience, not a lifeline.
        The hardware kill switch is the real safety system.
        """
        if self.dry_run or not self.armed_by_us:
            return
        self.get_logger().warn("shutdown while armed - neutral + disarm")

        # Zeros first, directly on the publisher. No spinning, no services,
        # nothing that can block. Publish many: UDP drops packets.
        for _ in range(20):
            try:
                self.neutral()
            except Exception:  # noqa: BLE001
                break
            time.sleep(0.02)

        # Then try to disarm, with retries, tolerating a torn-down context.
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

    # Ctrl+C sets the abort flag rather than unwinding the stack mid-service-
    # call. guard() sees it on the next tick and aborts cleanly through the
    # normal path. A second Ctrl+C falls through to KeyboardInterrupt.
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
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
