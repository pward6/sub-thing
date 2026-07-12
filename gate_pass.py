#!/usr/bin/env python3
"""
gate_pass.py - Nautilus autonomous gate pass.

Sequence:
    WAIT_FCU -> WAIT_HEADING -> ARM -> DEPTH_HOLD -> DIVE -> RUN -> DISARM

Holds a compass heading with a PI loop on Nucleus INS heading while driving
forward at a fixed thrust for a fixed duration, then disarms.

Target heading is captured at arm time unless -p target_heading:=<deg> is given.

    ros2 run nautilus_auto gate_pass
    ros2 run nautilus_auto gate_pass --ros-args -p target_heading:=47.0 -p dry_run:=true

SAFETY: dry_run:=true does everything except arm and publish thrust. Use it on
the bench. The node disarms on Ctrl+C, on timeout, and on any exception.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from rosidl_runtime_py.utilities import get_message

INS_TOPIC = "/nucleus_node/ins_packets"

# Candidate field names for heading, in priority order. The Nucleus driver's
# message layout is not something we should hardcode blind -- probe it.
HEADING_FIELDS = ("heading", "yaw", "heading_deg", "ins_heading")

# ManualControl for ArduSub: x/y/z/r in [-1000, 1000], except z in [0, 1000]
# with 500 = neutral (no vertical demand) when in DEPTH_HOLD.
Z_NEUTRAL = 500.0


def wrap180(deg):
    """Wrap angle to [-180, 180). Heading error must take the short way round."""
    return (deg + 180.0) % 360.0 - 180.0


class GatePass(Node):
    def __init__(self):
        super().__init__("gate_pass")

        self.declare_parameter("target_heading", float("nan"))  # NaN = capture at arm
        self.declare_parameter("run_seconds", 12.0)
        self.declare_parameter("forward_thrust", 0.30)   # fraction of full
        self.declare_parameter("dive_seconds", 4.0)      # settle in DEPTH_HOLD before driving
        self.declare_parameter("kp", 6.0)                # deg error -> yaw units
        self.declare_parameter("ki", 0.4)
        self.declare_parameter("yaw_limit", 400.0)       # cap yaw authority
        self.declare_parameter("i_limit", 200.0)         # anti-windup
        self.declare_parameter("heading_timeout", 1.0)   # s without INS -> abort
        self.declare_parameter("dry_run", False)

        p = lambda n: self.get_parameter(n).value
        self.target = float(p("target_heading"))
        self.run_seconds = float(p("run_seconds"))
        self.fwd = float(p("forward_thrust")) * 1000.0
        self.dive_seconds = float(p("dive_seconds"))
        self.kp = float(p("kp"))
        self.ki = float(p("ki"))
        self.yaw_limit = float(p("yaw_limit"))
        self.i_limit = float(p("i_limit"))
        self.heading_timeout = float(p("heading_timeout"))
        self.dry_run = bool(p("dry_run"))

        if not 0.0 <= self.fwd <= 1000.0:
            self.get_logger().fatal("forward_thrust must be 0.0-1.0")
            raise SystemExit(1)

        self.state = None
        self.heading = None
        self.heading_stamp = 0.0
        self.heading_field = None
        self.integral = 0.0
        self.last_t = None
        self.armed_by_us = False
        self.phase = "INIT"

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.ctrl_pub = self.create_publisher(
            ManualControl, "/mavros/manual_control/send", 10
        )

        self._subscribe_ins(sensor_qos)

        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        if self.dry_run:
            self.get_logger().warn("DRY RUN - will not arm, will not publish thrust.")

    # ---------- INS type resolution ----------

    def _subscribe_ins(self, qos):
        """Resolve the INS message type at runtime rather than hardcoding it."""
        deadline = time.time() + 10.0
        msg_type = None
        while time.time() < deadline:
            for name, types in self.get_topic_names_and_types():
                if name == INS_TOPIC and types:
                    msg_type = types[0]
                    break
            if msg_type:
                break
            rclpy.spin_once(self, timeout_sec=0.2)

        if not msg_type:
            self.get_logger().fatal(
                f"{INS_TOPIC} not advertised. Is nautilus_up.sh running?"
            )
            raise SystemExit(1)

        self.get_logger().info(f"INS type: {msg_type}")
        self.create_subscription(get_message(msg_type), INS_TOPIC, self._on_ins, qos)

    def _on_ins(self, msg):
        if self.heading_field is None:
            for f in HEADING_FIELDS:
                if hasattr(msg, f):
                    self.heading_field = f
                    self.get_logger().info(f"Heading field: '{f}'")
                    break
            else:
                self.get_logger().fatal(
                    "No heading field on INS message. Fields present: "
                    + ", ".join(msg.get_fields_and_field_types().keys())
                )
                raise SystemExit(1)

        self.heading = float(getattr(msg, self.heading_field)) % 360.0
        self.heading_stamp = time.time()

    def _on_state(self, msg):
        self.state = msg

    # ---------- service helpers ----------

    def _call(self, client, req, what):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{what}: service unavailable")
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            self.get_logger().error(f"{what}: call timed out")
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

    # ---------- control ----------

    def publish(self, x, r, z=Z_NEUTRAL):
        m = ManualControl()
        m.header.stamp = self.get_clock().now().to_msg()
        m.x = float(max(-1000.0, min(1000.0, x)))   # forward
        m.y = 0.0                                    # lateral
        m.z = float(max(0.0, min(1000.0, z)))        # vertical (500 = hold)
        m.r = float(max(-1000.0, min(1000.0, r)))    # yaw
        m.buttons = 0
        if not self.dry_run:
            self.ctrl_pub.publish(m)

    def neutral(self):
        self.publish(0.0, 0.0)

    def yaw_correction(self):
        """PI on heading error. Returns yaw demand, or None if heading is stale."""
        if self.heading is None or (time.time() - self.heading_stamp) > self.heading_timeout:
            return None

        err = wrap180(self.target - self.heading)

        now = time.time()
        dt = 0.0 if self.last_t is None else now - self.last_t
        self.last_t = now

        if dt > 0.0:
            self.integral += err * dt
            self.integral = max(-self.i_limit, min(self.i_limit, self.integral))

        u = self.kp * err + self.ki * self.integral
        return max(-self.yaw_limit, min(self.yaw_limit, u))

    # ---------- sequence ----------

    def spin_until(self, pred, timeout, what):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
        self.get_logger().error(f"Timed out waiting for {what}")
        return False

    def run(self):
        self.phase = "WAIT_FCU"
        self.get_logger().info("Waiting for FCU connection...")
        if not self.spin_until(
            lambda: self.state is not None and self.state.connected, 30.0, "connected:true"
        ):
            return 1

        self.phase = "WAIT_HEADING"
        self.get_logger().info("Waiting for INS heading...")
        if not self.spin_until(lambda: self.heading is not None, 20.0, "INS heading"):
            return 1

        if math.isnan(self.target):
            self.target = self.heading
            self.get_logger().info(f"Target heading CAPTURED at {self.target:.1f} deg")
        else:
            self.get_logger().info(
                f"Target heading from param: {self.target:.1f} deg "
                f"(current {self.heading:.1f})"
            )

        self.phase = "ARM"
        self.get_logger().info("Arming...")
        self.neutral()
        if not self.arm(True):
            self.get_logger().fatal("Arm rejected. SYSID_MYGCS=1? Safety switch?")
            return 1

        self.phase = "DEPTH_HOLD"
        if not self.set_mode("ALT_HOLD"):
            self.get_logger().fatal("Mode change rejected.")
            self.arm(False)
            return 1
        self.get_logger().info("DEPTH_HOLD engaged.")

        # ---- DIVE: hold heading, no forward thrust, let depth settle ----
        self.phase = "DIVE"
        self.get_logger().info(f"Settling {self.dive_seconds:.0f}s...")
        # NOTE: rclpy Rate.sleep() deadlocks without a separate spin thread.
        # Single-threaded loop -> plain sleep.
        DT = 0.05  # 20 Hz
        t_end = time.time() + self.dive_seconds
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.0)
            u = self.yaw_correction()
            if u is None:
                self.get_logger().error("INS heading STALE during dive. Aborting.")
                return self.abort()
            self.publish(0.0, u)
            time.sleep(DT)

        # ---- RUN: forward at fixed thrust, holding heading ----
        self.phase = "RUN"
        self.get_logger().info(
            f"RUN: {self.run_seconds:.0f}s at thrust {self.fwd/1000:.2f}, "
            f"holding {self.target:.1f} deg"
        )
        DT = 0.05
        t_end = time.time() + self.run_seconds
        last_log = 0.0
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.0)
            u = self.yaw_correction()
            if u is None:
                self.get_logger().error("INS heading STALE during run. Aborting.")
                return self.abort()
            self.publish(self.fwd, u)

            if time.time() - last_log > 1.0:
                last_log = time.time()
                err = wrap180(self.target - self.heading)
                self.get_logger().info(
                    f"  hdg {self.heading:6.1f}  err {err:+6.1f}  yaw {u:+7.1f}  "
                    f"t-{t_end - time.time():4.1f}"
                )
            time.sleep(DT)

        self.phase = "DISARM"
        self.get_logger().info("Run complete. Coasting, then disarming.")
        for _ in range(20):
            self.neutral()
            rclpy.spin_once(self, timeout_sec=0.05)
        self.arm(False)
        self.get_logger().info("Disarmed. Gate pass sequence finished.")
        return 0

    def abort(self):
        self.get_logger().warn(f"ABORT from phase {self.phase}")
        for _ in range(10):
            self.neutral()
            time.sleep(0.05)
        self.arm(False)
        return 1

    def safe_shutdown(self):
        if self.armed_by_us and not self.dry_run:
            self.get_logger().warn("Shutdown while armed - neutral + disarm.")
            for _ in range(10):
                self.neutral()
                time.sleep(0.05)
            self.arm(False)


def main():
    rclpy.init()
    node = GatePass()
    rc = 1
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl+C")
        node.safe_shutdown()
    except Exception as e:  # noqa: BLE001
        node.get_logger().fatal(f"Unhandled: {e}")
        node.safe_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
