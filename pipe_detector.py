#!/usr/bin/env python3
"""
pipe_detector.py - ROS2/ZED vision node: gate + pole, with its own
confirm-streak state tracking.

Publishes JSON on /nautilus/detections, ~10 Hz:

    {"t": 1783571583.9,
     "gate": {"bearing_deg": -3.2, "range_m": 5.4, "range_ok": true,
              "conf": 0.82, "confirmed": true, "streak": 6, "age_s": 0.1},
     "pole": {...same shape...}}

A missing key means that target has never been confirmed. "confirmed" only
goes true once a run of consecutive sightings agree on bearing -- a single
frame is not enough to trust (see tracking.Tracker). "age_s" is how long
ago it was ACTUALLY seen in a frame, not how long ago this message was
published: the tracker HOLDS the last confirmed reading through a brief
miss (a wave, a bad frame) so a consumer doesn't need its own jitter logic,
but it does not hold forever -- age_s is what lets a consumer (qualify.py)
tell "stabilized" apart from "stale, camera hasn't actually seen this in
30 seconds." Trust age_s, not just message arrival time.

Confirmation keys on BEARING, not range: bearing is available at every
distance, stereo range only inside vision_range_min..max. A gate steering
decision that only worked once the vehicle was already within stereo range
would be useless for the approach itself.

The gate is no longer a "pair of verticals, maybe synthesized" -- see
gate_shape.py: it is only ever reported when a real crossbar was traced to
two real legs, so there is no separate "was this a matched pair vs a
guess" flag to check anymore.

PIPELINE (see gate_shape.py for the algorithm + its tuning constants)
    grayscale -> gate_shape.find_gate_shape / find_pole_shape
             -> pixel corners/segments -> bearing via calibrated fx_eff
             -> range via ZED stereo depth, refraction-corrected
             -> tracking.Tracker per target: confirm-streak + hold-through-miss

OPTICS -- FLAT PORT
    Underwater a flat port raises effective focal length by ~n_water (1.33).
    The SDK computes depth with the AIR focal length, so it reads SHORT: a
    true 4.0 m reports ~3.0 m. Ranges are multiplied by refraction_scale
    before publication. Bearings use fx_eff = fx_air * refraction_scale.
    The defaults are theory -- MEASURE THEM: submerge, put a target at a
    tape-measured distance, read what the SDK reports, take the ratio.

    ros2 run nautilus_auto pipe_detector --ros-args -p show:=true
    ros2 run nautilus_auto pipe_detector --ros-args -p refraction_scale:=1.0
"""

import json
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gate_shape as shape
from tracking import Tracker

try:
    import pyzed.sl as sl
except ImportError:
    sys.exit("pyzed not found. Source the ZED SDK python API.")


class PipeDetector(Node):

    def __init__(self):
        super().__init__("pipe_detector")

        d = self.declare_parameter
        d("depth_mode", "NEURAL")
        d("resolution", "HD720")
        d("refraction_scale", 1.33)      # 1.0 when dry. MEASURE THIS.
        d("vision_range_max", 8.0)       # m, beyond this stereo is noise
        d("vision_range_min", 0.4)

        d("gate_confirm_frames", 6)
        d("gate_bearing_jitter_deg", 12.0)
        d("pole_confirm_frames", 3)
        d("pole_bearing_jitter_deg", 15.0)
        d("hold_timeout", 3.0)      # s a confirmed target may go unseen before it's dropped

        d("rate_hz", 10.0)
        d("show", False)            # local cv2.imshow debug windows

        g = lambda n: self.get_parameter(n).value
        self.scale = float(g("refraction_scale"))
        self.rmax = float(g("vision_range_max"))
        self.rmin = float(g("vision_range_min"))
        self.hold_timeout = float(g("hold_timeout"))
        self.period = 1.0 / float(g("rate_hz"))
        self.show = bool(g("show"))

        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, str(g("resolution")))
        init.depth_mode = getattr(sl.DEPTH_MODE, str(g("depth_mode")))
        init.coordinate_units = sl.UNIT.METER
        init.depth_minimum_distance = 0.3
        init.depth_maximum_distance = 40.0
        # Pool water has almost no texture. Self-calibration against it makes
        # depth worse, not better. Calibrate once on a cluttered dry scene.
        init.camera_disable_self_calib = True

        self.cam = sl.Camera()
        st = self.cam.open(init)
        if st != sl.ERROR_CODE.SUCCESS:
            self.get_logger().fatal(f"ZED open failed: {st}")
            raise SystemExit(1)

        info = self.cam.get_camera_information()
        cfg = info.camera_configuration
        self.w = cfg.resolution.width
        self.h = cfg.resolution.height
        fx_air = cfg.calibration_parameters.left_cam.fx
        self.fx = fx_air * self.scale       # effective focal length, wet

        self.get_logger().info(
            f"{self.w}x{self.h}  fx_air {fx_air:.1f} -> fx_eff {self.fx:.1f}  "
            f"scale {self.scale:.2f}  range {self.rmin:.1f}-{self.rmax:.1f}m")

        self.img = sl.Mat()
        self.depth = sl.Mat()
        self.rt = sl.RuntimeParameters()
        self.pub = self.create_publisher(String, "/nautilus/detections", 10)

        self.gate_tracker = Tracker(key="bearing_deg",
                                    confirm_frames=int(g("gate_confirm_frames")),
                                    epsilon=float(g("gate_bearing_jitter_deg")))
        self.pole_tracker = Tracker(key="bearing_deg",
                                    confirm_frames=int(g("pole_confirm_frames")),
                                    epsilon=float(g("pole_bearing_jitter_deg")))
        self.gate_last_seen = 0.0
        self.pole_last_seen = 0.0

    # ---------------- optics ----------------

    def bearing(self, x):
        """Pixel column -> degrees, + is right of centre. Pinhole, fx_eff."""
        return float(np.degrees(np.arctan2(x - self.w / 2.0, self.fx)))

    def range_at_points(self, dmap, pts, half=6):
        """Median finite depth sampled along a traced leg/segment, subsampled
        to ~6 points, refraction-corrected. (range_m, ok)."""
        step = max(1, len(pts) // 6)
        samples = []
        for x, y in pts[::step]:
            x0, x1 = max(0, int(x) - half), min(self.w, int(x) + half)
            y0, y1 = max(0, int(y) - half), min(self.h, int(y) + half)
            patch = dmap[y0:y1, x0:x1]
            good = patch[np.isfinite(patch)]
            if good.size:
                samples.append(good)
        if not samples or sum(a.size for a in samples) < 8:
            return 0.0, False
        z = float(np.median(np.concatenate(samples))) * self.scale
        return round(z, 2), bool(self.rmin <= z <= self.rmax)

    # ---------------- per-target readings ----------------

    def _gate_reading(self, gray, residual, dmap):
        """-> (reading dict or None, [(x_lo,x_hi), ...] leg columns to
        exclude from pole detection)."""
        gate = shape.find_gate_shape(gray, residual)
        if gate is None:
            return None, ()

        rng, ok = self.range_at_points(dmap, gate["left_leg"] + gate["right_leg"])
        x1, y1, x2, y2 = gate["crossbar"]
        cx = (x1 + x2) / 2.0
        reading = {"bearing_deg": round(self.bearing(cx), 2), "range_m": rng,
                  "range_ok": ok, "conf": gate["conf"]}
        exclude = [(min(x1, x2) - 40, min(x1, x2) + 40),
                  (max(x1, x2) - 40, max(x1, x2) + 40)]
        return reading, exclude

    def _pole_reading(self, gray, dmap, exclude):
        pole = shape.find_pole_shape(gray, exclude)
        if pole is None:
            return None
        x1, y1, x2, y2 = pole["segment"]
        pts = [(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t) for t in np.linspace(0, 1, 8)]
        rng, ok = self.range_at_points(dmap, pts)
        return {"bearing_deg": round(self.bearing((x1 + x2) / 2.0), 2), "range_m": rng,
               "range_ok": ok, "conf": pole["conf"]}

    def _target_json(self, tracker, last_seen, now):
        if tracker.confirmed is None:
            return None
        age = now - last_seen
        if age > self.hold_timeout:
            return None
        out = dict(tracker.confirmed)
        out["confirmed"] = tracker.streak >= tracker.confirm_frames
        out["streak"] = tracker.streak
        out["age_s"] = round(age, 2)
        return out

    # ---------------- debug rendering ----------------

    def annotate(self, bgr, det):
        vis = bgr.copy()
        if "gate" in det:
            g = det["gate"]
            tag = "OK" if g["confirmed"] else f"{g['streak']}/{self.gate_tracker.confirm_frames}"
            cv2.putText(vis, f"GATE b{g['bearing_deg']:+.1f} "
                             f"r{g['range_m'] if g['range_ok'] else '--'} {tag}",
                        (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 80, 255), 2)
        if "pole" in det:
            p = det["pole"]
            cv2.putText(vis, f"POLE b{p['bearing_deg']:+.1f} "
                             f"r{p['range_m'] if p['range_ok'] else '--'}",
                        (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 220, 255), 2)
        return vis

    # ---------------- loop ----------------

    def spin(self):
        nxt = time.time()
        while rclpy.ok():
            if self.cam.grab(self.rt) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.02)
                continue
            self.cam.retrieve_image(self.img, sl.VIEW.LEFT)
            self.cam.retrieve_measure(self.depth, sl.MEASURE.DEPTH)

            bgr = np.ascontiguousarray(self.img.get_data()[:, :, :3])
            dmap = self.depth.get_data()
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            residual = shape.background_residual(gray)

            now = time.time()
            gate_reading, exclude = self._gate_reading(gray, residual, dmap)
            if gate_reading:
                self.gate_last_seen = now
            self.gate_tracker.update(gate_reading)

            pole_reading = self._pole_reading(gray, dmap, exclude)
            if pole_reading:
                self.pole_last_seen = now
            self.pole_tracker.update(pole_reading)

            det = {"t": round(now, 3)}
            g = self._target_json(self.gate_tracker, self.gate_last_seen, now)
            p = self._target_json(self.pole_tracker, self.pole_last_seen, now)
            if g:
                det["gate"] = g
            if p:
                det["pole"] = p

            m = String()
            m.data = json.dumps(det)
            self.pub.publish(m)

            if self.show:
                cv2.imshow("detections", self.annotate(bgr, det))
                if cv2.waitKey(1) == 27:
                    rclpy.shutdown()
                    break

            rclpy.spin_once(self, timeout_sec=0.0)
            nxt += self.period
            time.sleep(max(0.0, nxt - time.time()))

    def close(self):
        self.cam.close()
        if self.show:
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = PipeDetector()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
