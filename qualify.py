#!/usr/bin/env python3

"""
qualify.py - Nautilus gate run.

Mission:
    Look for the gate -> seek altitude -> settle -> drive through gate ->
    stop horizontal movement while commanding z=450 -> remain stopped until
    an abort request or the mission timeout.

Depth/thrust behavior:
- SEEK_ALTITUDE uses STABILIZE so the z channel directly drives the vertical
  thrusters while the altimeter is used to reach TARGET_ALTITUDE_M.
- After SEEK_ALTITUDE, the vehicle switches to ALT_HOLD for settling and
  driving through the gate.
- STOP switches back to STABILIZE and continuously sends:
      x = 0
      y = 0
      r = 0
      z = STOP_HOLD_Z
  where STOP_HOLD_Z is 450, corresponding to the requested 0.45 command.

Vision:
- pipe_detector.py publishes gate detections on /nautilus/detections.
- The vehicle waits for a confirmed gate before arming unless
  skip_gate_wait:=true is used for bench testing.

Useful test commands:
    ros2 run nautilus_auto qualify --ros-args -p dry_run:=true
    ros2 run nautilus_auto qualify --ros-args -p fake_gate:=true

PRE-FLIGHT:
Confirm in water that velocity_nucleus_x changes and fom_ins drops below
approximately 5. On the bench, the INS position solution may be meaningless.
Use require_bottom_lock:=true to require a valid bottom lock before launch.
"""

import json
import math
import os
import signal
import sys
import time

import rclpy
from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Empty, String


INS_TOPIC = "/nucleus_node/ins_packets"
ALTIMETER_TOPIC = "/nucleus_node/altimeter_packets"

Z_NEUTRAL = 500.0
STOP_HOLD_Z = 450.0

DT = 0.05
FAKE_GATE_SECONDS = 20.0
FAKE_GATE_START_RANGE = 8.0

# Altitude-seeking settings
TARGET_ALTITUDE_M = 0.5
ALTITUDE_TOLERANCE_M = 0.1
ALTITUDE_KP = 300.0
ALTITUDE_Z_MAX = 200.0
ALTITUDE_TIMEOUT = 120.0
ALTIMETER_TIMEOUT = 3.0
ALTITUDE_MIN_SAFE_M = 0.0
ALTITUDE_SIGN = 1.0
ALTITUDE_STALL_CHECK_S = 8.0
ALTITUDE_STALL_MIN_MOVE_M = 0.03

# Heading and mission settings
KP_HEADING, KI_HEADING = 6.0, 0.4
YAW_LIMIT, I_LIMIT = 400.0, 200.0
VISION_TIMEOUT = 1.5
SETTLE_SECONDS = 3.0
GATE_OVERSHOOT = 2.5
GATE_PASS_RANGE = 1.6
GATE_LOST_STOP = 5.0
SCAN_YAW = 150.0
SEARCH_TIMEOUT = 90.0
GATE_MIN_CONF = 0.20
INS_TIMEOUT = 1.0
STATE_MAX_AGE = 5.0

CMD_LOG_DIR = "~/nautilus_ws/logs"


def wrap180(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class Qualify(Node):
    def __init__(self):
        super().__init__("qualify")

        declare = self.declare_parameter
        declare("dry_run", False)
        declare("fake_gate", False)
        declare("fake_gate_bearing_deg", 0.0)
        declare("fake_gate_range_m", 3.0)
        declare("target_heading", float("nan"))
        declare("cruise_speed", 0.35)
        declare("arm_delay", 5.0)
        declare("mission_timeout", 1000.0)
        declare("gate_acquire_timeout", 0.0)
        declare("skip_gate_wait", False)
        declare("require_bottom_lock", False)
        declare("max_fom_ins", 10.0)

        get = lambda name: self.get_parameter(name).value

        self.dry_run = bool(get("dry_run"))
        self.fake_gate = bool(get("fake_gate"))
        self.fake_bearing = float(get("fake_gate_bearing_deg"))
        self.fake_range = float(get("fake_gate_range_m"))
        self.gate_heading = float(get("target_heading"))
        self.cruise_v = float(get("cruise_speed")) * 1000.0
        self.arm_delay = float(get("arm_delay"))
        self.mission_timeout = float(get("mission_timeout"))
        self.acquire_timeout = float(get("gate_acquire_timeout"))
        self.require_lock = bool(get("require_bottom_lock"))
        self.max_fom = float(get("max_fom_ins"))

        self._fake_until = None

        # INS state
        self.heading = None
        self.pos = None
        self.vel_x = 0.0
        self.fom_ins = 999.0
        self.ins_stamp = 0.0

        # Altimeter state
        self.altimeter_distance = None
        self.altimeter_stamp = 0.0

        # Mission state
        self.origin = None
        self.armed_by_us = False
        self.phase = "INIT"
        self.mission_start = None

        # Heading PI controller
        self.integral = 0.0
        self.last_t = None

        # Vision state
        self._gate = None
        self._log_times = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._cmd_log = self._open_cmd_log()
        self._run_log = self._open_run_log()

        self.create_subscription(
            State,
            "/mavros/state",
            self._on_state,
            self._match_qos("/mavros/state"),
        )

        self.ctrl = self.create_publisher(
            ManualControl,
            "/mavros/manual_control/send",
            10,
        )

        self._subscribe_ins(qos)
        self._subscribe_altimeter(qos)

        self.create_subscription(
            String,
            "/nautilus/detections",
            self._on_detections,
            10,
        )

        self.create_subscription(
            Empty,
            "/nautilus/cmd/abort",
            self._on_abort,
            10,
        )

        self._abort_req = False

        self.arm_cli = self.create_client(
            CommandBool,
            "/mavros/cmd/arming",
        )
        self.mode_cli = self.create_client(
            SetMode,
            "/mavros/set_mode",
        )

        self.state = None
        self.state_stamp = 0.0

        if self.dry_run:
            self._log("warn", "DRY RUN: no arm, no thrust published.")

        if self.fake_gate:
            self._log(
                "warn",
                (
                    "FAKE_GATE: gate faked CONFIRMED at bearing "
                    f"{self.fake_bearing:+.1f} deg, range simulated closing "
                    f"{FAKE_GATE_START_RANGE:.1f}m -> {self.fake_range:.1f}m "
                    f"over {FAKE_GATE_SECONDS:.0f}s, starting the first time "
                    "it is checked."
                ),
            )

    # ---------------- logging ----------------

    def _open_cmd_log(self):
        path = os.path.join(
            os.path.expanduser(CMD_LOG_DIR),
            f"qualify_cmds_{time.strftime('%Y%m%d_%H%M%S')}.csv",
        )
        os.makedirs(os.path.dirname(path), exist_ok=True)

        file = open(path, "w", buffering=1)
        file.write("t,phase,x,y,z,r\n")

        self.get_logger().info(f"command log: {path}")
        return file

    def _open_run_log(self):
        path = os.path.join(
            os.path.expanduser(CMD_LOG_DIR),
            f"qualify_run_{time.strftime('%Y%m%d_%H%M%S')}.log",
        )
        os.makedirs(os.path.dirname(path), exist_ok=True)

        file = open(path, "w", buffering=1)
        self.get_logger().info(f"run log: {path}")
        return file

    def _log(self, level, message):
        logger = self.get_logger()

        if level == "info":
            logger.info(message)
        elif level == "warn":
            logger.warn(message)
        elif level == "error":
            logger.error(message)
        else:
            logger.fatal(message)

        self._run_log.write(
            f"{time.time():.3f} "
            f"[{level.upper():5s}] "
            f"{self.phase:16s} "
            f"{message}\n"
        )

    def _log_cmd(self, x, y, z, r):
        self._cmd_log.write(
            f"{time.time():.3f},"
            f"{self.phase},"
            f"{x:.1f},"
            f"{y:.1f},"
            f"{z:.1f},"
            f"{r:.1f}\n"
        )

    def close_logs(self):
        for file in (self._cmd_log, self._run_log):
            try:
                file.close()
            except Exception:  # noqa: BLE001
                pass

    # ---------------- subscriptions ----------------

    def _match_qos(self, topic, timeout=8.0):
        deadline = time.time() + timeout

        while time.time() < deadline:
            infos = self.get_publishers_info_by_topic(topic)

            if infos:
                qos = infos[0].qos_profile

                self._log(
                    "info",
                    (
                        f"{topic}: matching pub QoS "
                        f"{qos.reliability.name}/{qos.durability.name}"
                    ),
                )

                return QoSProfile(
                    reliability=qos.reliability,
                    durability=qos.durability,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                )

            rclpy.spin_once(self, timeout_sec=0.2)

        self._log("warn", f"{topic}: no publisher, using default QoS")

        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _subscribe_ins(self, qos):
        deadline = time.time() + 10.0
        message_type = None

        while time.time() < deadline and not message_type:
            for name, types in self.get_topic_names_and_types():
                if name == INS_TOPIC and types:
                    message_type = types[0]
                    break

            rclpy.spin_once(self, timeout_sec=0.2)

        if not message_type:
            self._log(
                "fatal",
                f"{INS_TOPIC} absent. nautilus_up.sh running?",
            )
            raise SystemExit(1)

        self._log("info", f"INS type: {message_type}")

        self.create_subscription(
            get_message(message_type),
            INS_TOPIC,
            self._on_ins,
            qos,
        )

    def _subscribe_altimeter(self, qos):
        deadline = time.time() + 10.0
        message_type = None

        while time.time() < deadline and not message_type:
            for name, types in self.get_topic_names_and_types():
                if name == ALTIMETER_TOPIC and types:
                    message_type = types[0]
                    break

            rclpy.spin_once(self, timeout_sec=0.2)

        if not message_type:
            self._log(
                "fatal",
                f"{ALTIMETER_TOPIC} absent. nautilus_up.sh running?",
            )
            raise SystemExit(1)

        self._log("info", f"altimeter type: {message_type}")

        self.create_subscription(
            get_message(message_type),
            ALTIMETER_TOPIC,
            self._on_altimeter,
            qos,
        )

    # ---------------- callbacks ----------------

    def _on_state(self, message):
        self.state = message
        self.state_stamp = time.time()

    def _on_abort(self, _message):
        self._log(
            "warn",
            "ABORT REQUESTED on /nautilus/cmd/abort",
        )
        self._abort_req = True

    def fcu_live(self):
        if self.state is None or not self.state.connected:
            return False

        stamp = self.state.header.stamp
        message_time = stamp.sec + stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9

        return (now - message_time) <= STATE_MAX_AGE

    def _on_ins(self, message):
        self.heading = float(message.heading) % 360.0
        self.pos = (
            float(message.position_frame_x),
            float(message.position_frame_y),
        )
        self.vel_x = float(message.velocity_nucleus_x)
        self.fom_ins = float(message.fom_ins)
        self.ins_stamp = time.time()

    def _on_altimeter(self, message):
        self.altimeter_distance = float(message.altimeter_distance)
        self.altimeter_stamp = time.time()

    def _on_detections(self, message):
        try:
            data = json.loads(message.data)
        except (ValueError, TypeError):
            return

        if data.get("gate"):
            self._gate = dict(
                data["gate"],
                recv_t=time.time(),
            )

    # ---------------- gate data ----------------

    def _gate_record(self):
        if self.fake_gate:
            if self._fake_until is None:
                self._fake_until = time.time() + FAKE_GATE_SECONDS

            remaining = self._fake_until - time.time()

            if remaining > 0:
                fraction = 1.0 - remaining / FAKE_GATE_SECONDS
                start = max(
                    FAKE_GATE_START_RANGE,
                    self.fake_range,
                )
                simulated_range = (
                    start
                    + (self.fake_range - start) * fraction
                )

                return {
                    "bearing_deg": self.fake_bearing,
                    "range_m": round(simulated_range, 2),
                    "range_ok": True,
                    "conf": 1.0,
                    "confirmed": True,
                }

        if not self._gate:
            return None

        total_age = (
            self._gate["age_s"]
            + time.time()
            - self._gate["recv_t"]
        )

        if total_age < VISION_TIMEOUT:
            return self._gate

        return None

    def gate_bearing(self):
        record = self._gate_record()

        if record:
            return record["bearing_deg"]

        return None

    def gate_range(self):
        record = self._gate_record()

        if record:
            return record["range_m"], record["range_ok"]

        return None, False

    def gate_conf(self):
        record = self._gate_record()

        if record:
            return record["conf"]

        return 0.0

    def gate_is_solid(self):
        record = self._gate_record()

        return bool(
            record
            and record.get("confirmed")
            and record["conf"] >= GATE_MIN_CONF
        )

    # ---------------- geometry ----------------

    def along_cross(self):
        dx = self.pos[0] - self.origin[0]
        dy = self.pos[1] - self.origin[1]

        heading_rad = math.radians(self.gate_heading)

        along = (
            dx * math.cos(heading_rad)
            + dy * math.sin(heading_rad)
        )
        cross = (
            -dx * math.sin(heading_rad)
            + dy * math.cos(heading_rad)
        )

        return along, cross

    def ins_ok(self):
        if self.heading is None or self.pos is None:
            return False

        if time.time() - self.ins_stamp > INS_TIMEOUT:
            return False

        if self.require_lock and self.fom_ins > self.max_fom:
            return False

        return True

    # ---------------- actuation ----------------

    def publish(self, x, r, z=Z_NEUTRAL, y=0.0):
        message = ManualControl()
        message.header.stamp = self.get_clock().now().to_msg()

        message.x = float(clamp(x, -1000, 1000))
        message.y = float(clamp(y, -1000, 1000))
        message.z = float(clamp(z, 0, 1000))
        message.r = float(clamp(r, -1000, 1000))
        message.buttons = 0

        self._log_cmd(
            message.x,
            message.y,
            message.z,
            message.r,
        )

        if not self.dry_run:
            self.ctrl.publish(message)

    def neutral(self):
        self.publish(0.0, 0.0)

    def yaw_to(self, target_heading):
        error = wrap180(target_heading - self.heading)

        now = time.time()
        dt = 0.0 if self.last_t is None else now - self.last_t
        self.last_t = now

        if dt > 0.0:
            self.integral = clamp(
                self.integral + error * dt,
                -I_LIMIT,
                I_LIMIT,
            )

        return clamp(
            KP_HEADING * error + KI_HEADING * self.integral,
            -YAW_LIMIT,
            YAW_LIMIT,
        )

    def reset_pi(self):
        self.integral = 0.0
        self.last_t = None

    # ---------------- MAVROS services ----------------

    def _call(self, client, request, description):
        if not client.wait_for_service(timeout_sec=5.0):
            self._log(
                "error",
                f"{description}: service unavailable",
            )
            return None

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        return future.result()

    def arm(self, value):
        if self.dry_run:
            self._log("info", f"[dry_run] arm({value})")
            return True

        request = CommandBool.Request()
        request.value = value

        response = self._call(
            self.arm_cli,
            request,
            "arm",
        )

        success = bool(response and response.success)

        if success and value:
            self.armed_by_us = True

        return success

    def set_mode(self, mode):
        if self.dry_run:
            self._log(
                "info",
                f"[dry_run] set_mode({mode})",
            )
            return True

        request = SetMode.Request()
        request.custom_mode = mode

        response = self._call(
            self.mode_cli,
            request,
            "set_mode",
        )

        return bool(response and response.mode_sent)

    # ---------------- loop helpers ----------------

    def guard(self):
        if self._abort_req:
            return "abort requested"

        if not self.ins_ok():
            return (
                "INS unusable "
                f"(stale, or fom_ins={self.fom_ins:.1f})"
            )

        if (
            self.mission_start
            and time.time() - self.mission_start
            > self.mission_timeout
        ):
            return "mission timeout"

        return None

    def tick(self, x, r, z=Z_NEUTRAL, y=0.0):
        rclpy.spin_once(self, timeout_sec=0.0)

        reason = self.guard()

        if reason:
            raise Abort(reason)

        self.publish(x, r, z, y)
        time.sleep(DT)

    def spin_until(self, predicate, timeout, description):
        deadline = time.time() + timeout

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if predicate():
                return True

        self._log(
            "error",
            f"timeout waiting for {description}",
        )
        return False

    def enter(self, phase):
        self.phase = phase
        self.reset_pi()
        self._log("info", f"=== {phase} ===")

    def log_every(self, key, interval, message_function):
        now = time.time()

        if now - self._log_times.get(key, 0.0) > interval:
            self._log_times[key] = now
            self._log("info", message_function())

    # ---------------- mission ----------------

    def run(self):
        self.enter("WAIT_FCU")

        if not self.spin_until(
            self.fcu_live,
            30.0,
            "a fresh connected:true",
        ):
            if self.state is None:
                self._log(
                    "error",
                    (
                        "no /mavros/state at all. "
                        "Is MAVROS running? "
                        "pgrep -af mavros_node"
                    ),
                )
            elif not self.state.connected:
                self._log(
                    "error",
                    (
                        "MAVROS up, FCU silent. "
                        "Cube powered? TELEM2 harness?"
                    ),
                )
            else:
                self._log(
                    "error",
                    (
                        "connected:true but STALE - that is a "
                        "latched message from a dead MAVROS. "
                        "Check: ros2 topic hz /mavros/state"
                    ),
                )

            return 1

        self.enter("WAIT_INS")

        if not self.spin_until(
            lambda: self.pos is not None,
            20.0,
            "INS",
        ):
            return 1

        self._log(
            "info",
            (
                f"hdg {self.heading:.1f} "
                f"fom_ins {self.fom_ins:.1f} "
                f"vel_x {self.vel_x:.3f} "
                f"pos {self.pos[0]:.2f},{self.pos[1]:.2f}"
            ),
        )

        if self.fom_ins > self.max_fom:
            message = (
                f"fom_ins={self.fom_ins:.1f} > "
                f"{self.max_fom} - no bottom lock. "
                "Dead reckoning will drift."
            )

            if self.require_lock:
                self._log("fatal", message)
                return 1

            self._log(
                "warn",
                message + " Proceeding anyway.",
            )

        if not self.wait_for_gate():
            return 1

        if math.isnan(self.gate_heading):
            self.gate_heading = self.heading
            self._log(
                "info",
                (
                    "Gate heading CAPTURED: "
                    f"{self.gate_heading:.1f}"
                ),
            )

        self.origin = self.pos

        self.enter("ARM")
        self.neutral()

        if not self.arm(True):
            self._log(
                "fatal",
                "arm rejected (SYSID_MYGCS=1?)",
            )
            return 1

        if not self.set_mode("STABILIZE"):
            self._log("fatal", "mode rejected")
            self.arm(False)
            return 1

        try:
            self.seek_altitude()
            self.settle()
            self.drive_to_gate()

            # Stop all horizontal motion and continuously command
            # the requested 0.45 vertical-thrust value.
            self.stop()

        except Abort as error:
            return self.abort(str(error))

        # This section is reached only when stop() is called with a
        # finite duration. With self.stop() and no duration, STOP
        # continues until an abort request or mission timeout.
        self.enter("DISARM")

        for _ in range(20):
            self.neutral()
            rclpy.spin_once(self, timeout_sec=DT)

        self.arm(False)
        self._log("info", "Run complete.")

        return 0

    def wait_for_gate(self):
        self.enter("WAIT_GATE")

        if bool(
            self.get_parameter("skip_gate_wait").value
        ):
            self._log(
                "warn",
                "skip_gate_wait: arming without a gate.",
            )
            self.mission_start = time.time()
            return True

        forever = self.acquire_timeout <= 0.0
        deadline = (
            None
            if forever
            else time.time() + self.acquire_timeout
        )

        if forever:
            self._log(
                "info",
                (
                    "Waiting indefinitely for the gate. "
                    "Disarmed, no thrust. Ctrl+C or "
                    "/nautilus/cmd/abort to stop."
                ),
            )

        while rclpy.ok():
            if self._abort_req:
                self._log(
                    "error",
                    "Aborted before arming.",
                )
                return False

            if deadline and time.time() > deadline:
                self._log(
                    "error",
                    "Gate never appeared. Not arming.",
                )
                return False

            rclpy.spin_once(self, timeout_sec=0.05)

            if self.ins_ok() and self.gate_is_solid():
                break

            def status():
                left = (
                    ""
                    if forever
                    else (
                        f" ({deadline - time.time():.0f}s left)"
                    )
                )
                bearing = self.gate_bearing()
                distance, range_ok = self.gate_range()
                reason = (
                    "none"
                    if bearing is None
                    else (
                        "weak "
                        f"(conf {self.gate_conf():.2f})"
                    )
                )

                return (
                    f" waiting for gate [{reason}] "
                    f"bearing {bearing} "
                    f"range {distance if range_ok else '--'} "
                    f"hdg {self.heading:.1f} "
                    f"fom {self.fom_ins:.1f}"
                    f"{left}"
                )

            self.log_every(
                "wait_gate",
                5.0,
                status,
            )
        else:
            return False

        bearing = self.gate_bearing()
        distance, range_ok = self.gate_range()

        self._log(
            "warn",
            (
                f"GATE ACQUIRED: bearing {bearing:+.1f}, "
                f"range {distance if range_ok else 'far'}. "
                f"ARMING IN {self.arm_delay:.0f}s -- "
                "HANDS OFF."
            ),
        )

        countdown_end = time.time() + self.arm_delay

        while time.time() < countdown_end:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._abort_req:
                self._log(
                    "error",
                    "Aborted during arm countdown.",
                )
                return False

            remaining = countdown_end - time.time()

            if (
                remaining > 0
                and abs(remaining - round(remaining)) < 0.06
            ):
                self._log(
                    "warn",
                    f" arming in {round(remaining)}...",
                )

        self.mission_start = time.time()
        return True

    def search(self, seen, description):
        self.enter(f"SEARCH_{description.upper()}")

        deadline = time.time() + SEARCH_TIMEOUT
        direction = 1.0
        flip_time = time.time() + 8.0

        while time.time() < deadline:
            if seen():
                self._log(
                    "info",
                    f"{description} reacquired.",
                )
                return

            if time.time() > flip_time:
                direction *= -1.0
                flip_time = time.time() + 16.0

            self.tick(
                0.0,
                SCAN_YAW * direction,
            )

        raise Abort(
            f"{description} not found during search"
        )

    def seek_altitude(self):
        self.enter("SEEK_ALTITUDE")

        if not self.spin_until(
            lambda: self.altimeter_distance is not None,
            10.0,
            "altimeter",
        ):
            raise Abort(
                "no altimeter data -- cannot seek altitude"
            )

        deadline = time.time() + ALTITUDE_TIMEOUT
        stall_reference_altitude = None
        stall_reference_time = None

        while True:
            if time.time() > deadline:
                raise Abort(
                    (
                        f"could not reach "
                        f"{TARGET_ALTITUDE_M:.2f}m altitude in "
                        f"{ALTITUDE_TIMEOUT:.0f}s"
                    )
                )

            altitude = self.altimeter_distance
            age = time.time() - self.altimeter_stamp

            if age > ALTIMETER_TIMEOUT:
                self.log_every(
                    "seek_altitude_stale",
                    1.0,
                    lambda: (
                        f" altimeter stale "
                        f"({age:.2f}s old, last reading "
                        f"{altitude:.2f}m) -- holding, waiting "
                        "for a fresh one"
                    ),
                )

                self.tick(
                    0.0,
                    self.yaw_to(self.gate_heading),
                    z=Z_NEUTRAL,
                )
                continue

            if altitude < ALTITUDE_MIN_SAFE_M:
                raise Abort(
                    (
                        f"altimeter {altitude:.2f}m < "
                        f"{ALTITUDE_MIN_SAFE_M}m safety floor"
                    )
                )

            error = TARGET_ALTITUDE_M - altitude

            if abs(error) <= ALTITUDE_TOLERANCE_M:
                self._log(
                    "info",
                    (
                        f"altitude {altitude:.2f}m, "
                        f"target {TARGET_ALTITUDE_M:.2f}m "
                        f"(within {ALTITUDE_TOLERANCE_M:.2f}m). "
                        "Switching back to ALT_HOLD."
                    ),
                )

                self.publish(
                    0.0,
                    self.yaw_to(self.gate_heading),
                    z=Z_NEUTRAL,
                )

                if not self.set_mode("ALT_HOLD"):
                    raise Abort(
                        (
                            "could not switch back to "
                            "ALT_HOLD after SEEK_ALTITUDE"
                        )
                    )

                return

            z_command = clamp(
                (
                    Z_NEUTRAL
                    + ALTITUDE_SIGN
                    * ALTITUDE_KP
                    * error
                ),
                Z_NEUTRAL - ALTITUDE_Z_MAX,
                Z_NEUTRAL + ALTITUDE_Z_MAX,
            )

            now = time.time()

            if stall_reference_altitude is None:
                stall_reference_altitude = altitude
                stall_reference_time = now

            moved = abs(
                altitude - stall_reference_altitude
            )
            elapsed = now - stall_reference_time

            if elapsed > 0.5:
                altitude_rate_mm_s = (
                    (altitude - stall_reference_altitude)
                    / elapsed
                    * 1000.0
                )
                rate = f"{altitude_rate_mm_s:+.0f}mm/s"
            else:
                rate = "measuring..."

            if elapsed > ALTITUDE_STALL_CHECK_S:
                if moved < ALTITUDE_STALL_MIN_MOVE_M:
                    self._log(
                        "warn",
                        (
                            f"commanding z={z_command:.0f} "
                            f"(delta "
                            f"{z_command - Z_NEUTRAL:+.0f}) "
                            f"for {elapsed:.0f}s but altitude "
                            f"only moved {moved * 1000:.0f}mm "
                            "-- not enough thrust? "
                            "ALTITUDE_SIGN backwards? "
                            "Consider raising ALTITUDE_KP/"
                            "ALTITUDE_Z_MAX."
                        ),
                    )

                stall_reference_altitude = altitude
                stall_reference_time = now

            self.log_every(
                "seek_altitude",
                1.0,
                lambda: (
                    f" altimeter {altitude:.2f}m "
                    f"target {TARGET_ALTITUDE_M:.2f}m "
                    f"error {error:+.2f}m "
                    f"rate {rate} "
                    f"age {age:.2f}s "
                    f"z {z_command:.0f} "
                    f"(neutral {Z_NEUTRAL:.0f}, "
                    f"delta "
                    f"{z_command - Z_NEUTRAL:+.0f})"
                ),
            )

            self.tick(
                0.0,
                self.yaw_to(self.gate_heading),
                z=z_command,
            )

    def settle(self):
        self.enter("SETTLE")

        for _ in range(
            int(SETTLE_SECONDS / DT)
        ):
            self.tick(
                0.0,
                self.yaw_to(self.gate_heading),
            )

    def stop(self, duration=None):
        """
        Stop all horizontal motion and command a constant z value of 450.

        duration=None:
            Remain in STOP until an abort request or mission timeout.

        duration=<seconds>:
            Hold the stop command for that many seconds, then return.
        """
        self.enter("STOP")

        if not self.set_mode("STABILIZE"):
            raise Abort(
                "could not switch to STABILIZE for stop"
            )

        end_time = (
            None
            if duration is None
            else time.time() + duration
        )

        while rclpy.ok():
            if (
                end_time is not None
                and time.time() >= end_time
            ):
                return

            self.tick(
                x=0.0,
                y=0.0,
                r=0.0,
                z=STOP_HOLD_Z,
            )

    def blind_push(self, heading, metres):
        start_along, _ = self.along_cross()

        while True:
            along, _ = self.along_cross()

            if along - start_along >= metres:
                return

            self.tick(
                self.cruise_v,
                self.yaw_to(heading),
                z=Z_NEUTRAL,
            )

    def drive_to_gate(self):
        self.enter("DRIVE_TO_GATE")

        ever_seen = False
        lost_since = None

        while True:
            along, _ = self.along_cross()

            if along > 15.0:
                raise Abort(
                    (
                        f"drove {along:.1f}m without "
                        "reaching the gate"
                    )
                )

            bearing = self.gate_bearing()
            distance, range_ok = self.gate_range()

            if bearing is None:
                if not ever_seen:
                    self.search(
                        lambda: (
                            self.gate_bearing() is not None
                        ),
                        "gate",
                    )
                    continue

                if lost_since is None:
                    lost_since = time.time()

                lost_for = time.time() - lost_since

                if lost_for > GATE_LOST_STOP:
                    state = (
                        f"stopped (lost {lost_for:.1f}s)"
                    )
                    self.tick(
                        0.0,
                        self.yaw_to(self.gate_heading),
                    )
                else:
                    state = (
                        f"coasting (lost {lost_for:.1f}s)"
                    )
                    self.tick(
                        self.cruise_v,
                        self.yaw_to(self.gate_heading),
                        z=Z_NEUTRAL,
                    )
            else:
                ever_seen = True
                lost_since = None
                state = "driving"

                setpoint = (
                    self.heading + bearing
                ) % 360.0

                self.tick(
                    self.cruise_v,
                    self.yaw_to(setpoint),
                    z=Z_NEUTRAL,
                )

                if (
                    range_ok
                    and distance < GATE_PASS_RANGE
                ):
                    break

            self.log_every(
                "drive_to_gate",
                2.0,
                lambda: (
                    f" [{state}] gate b "
                    f"{bearing if bearing is None else round(bearing, 1)} "
                    f"r {distance if range_ok else '--'} "
                    f"conf {self.gate_conf():.2f} "
                    f"along {along:+.2f} "
                    f"hdg {self.heading:.1f}"
                ),
            )

        self._log(
            "info",
            f"Gate at {distance:.2f}m. Pushing through.",
        )

        self.gate_heading = self.heading

        self.blind_push(
            self.gate_heading,
            GATE_OVERSHOOT,
        )

        self._log("info", "At the gate.")

    # ---------------- failure handling ----------------

    def abort(self, reason):
        self._log(
            "error",
            f"ABORT in {self.phase}: {reason}",
        )

        for _ in range(10):
            self.neutral()
            time.sleep(DT)

        self.arm(False)
        return 1

    def safe_shutdown(self):
        if self.dry_run or not self.armed_by_us:
            return

        self._log(
            "warn",
            "shutdown while armed - neutral + disarm",
        )

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
            except Exception as error:  # noqa: BLE001
                self._log(
                    "error",
                    (
                        f"disarm attempt "
                        f"{attempt + 1}: {error}"
                    ),
                )

            time.sleep(0.3)

        self._log(
            "error",
            (
                "COULD NOT DISARM. Hit the hardware "
                "kill switch. Then: ros2 service call "
                "/mavros/cmd/arming "
                "mavros_msgs/srv/CommandBool "
                "\"{value: false}\""
            ),
        )


class Abort(Exception):
    pass


def main():
    rclpy.init()
    node = Qualify()

    hits = {"n": 0}

    def on_sigint(_signal, _frame):
        hits["n"] += 1
        node._abort_req = True

        node.get_logger().warn(
            (
                "SIGINT: aborting"
                if hits["n"] == 1
                else "SIGINT again: forcing shutdown"
            )
        )

        if hits["n"] > 1:
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    return_code = 1

    try:
        return_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl+C")
        node.safe_shutdown()
    except Exception as error:  # noqa: BLE001
        node.get_logger().fatal(
            f"unhandled: {error}"
        )
        node.safe_shutdown()
    finally:
        node.close_logs()
        node.destroy_node()
        rclpy.shutdown()

    return return_code


if __name__ == "__main__":
    sys.exit(main())
