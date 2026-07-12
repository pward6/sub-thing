#!/usr/bin/env python3
"""
pipe_detector.py - Detect the PVC gate and the pole in the ZED Mini's view.

Publishes JSON on /nautilus/detections, ~10 Hz:

    {"t": 1783571583.9,
     "gate": {"bearing": -3.2, "range": 5.4, "range_ok": true,
              "crossbar_y": 0.31, "posts": [-11.4, 4.9], "conf": 0.82},
     "pole": {"bearing": 11.7, "range": 0.0, "range_ok": false, "conf": 0.55}}

A missing key means not detected this frame. `range_ok` false means the target
is outside the trusted range band -- ignore `range`, the bearing is still good.

WHY THIS IS NOT THE OLD DETECTOR
    The previous version isolated posts with MORPH_OPEN using a kernel ~0.16*h
    tall. Opening keeps only regions the structuring element fits INSIDE. A
    thin sunlit pipe threshholds into a DASHED stroke, and a 115 px kernel
    never fits inside a 20 px dash -- so the far post and the crossbar were
    erased before any size filter ran. That is a segmentation failure that
    looks like a classification failure.

    Line detection has the opposite failure mode. HoughLinesP's maxLineGap
    exists precisely to stitch collinear fragments, so a dashed pipe is still
    a strong line. We now:

      white mask -> anisotropic CLOSE (bridge dashes along each axis)
                 -> HoughLinesP      -> raw segments
                 -> merge collinear  -> one segment per physical pipe
                 -> split by angle   -> posts (vertical) / bars (horizontal)
                 -> depth gate       -> discard anything with no stereo support

    The depth gate is the important one. A lens-vignette corner, a sun glare
    patch, and a pool-wall tile all produce white pixels with NO valid
    disparity. Requiring that a candidate actually exist in 3D removes an
    entire class of phantom detection for two lines of code.

CLASSIFICATION (unchanged contract)
    A crossbar whose ends land on two posts identifies the GATE.
    Any remaining post is the POLE. With no crossbar, two posts at equal
    stereo depth are the gate; failing that, hysteresis holds the last gate
    briefly; failing that, the strongest lone post is the pole.

    Three verticals in frame is therefore not ambiguous: the crossbar decides.

OPTICS -- FLAT PORT
    Underwater a flat port raises effective focal length by ~n_water (1.33).
    The SDK computes depth with the AIR focal length, so it reads SHORT:
    a true 4.0 m reports ~3.0 m. Ranges are multiplied by refraction_scale
    before publication. Bearings use fx_eff = fx_air * refraction_scale.

    The defaults are theory. MEASURE THEM: submerge, put a target at a tape
    measured 2.0 m and again at 4.0 m, read centre depth, take
    scale = z_tape / z_sdk. If the two disagree by more than ~5%, the port
    is not a simple scale and we handle it differently.

    ros2 run nautilus_auto pipe_detector --ros-args -p show:=true
    ros2 run nautilus_auto pipe_detector --ros-args -p refraction_scale:=1.0
    ros2 run nautilus_auto pipe_detector --ros-args -p require_depth:=false
"""

import json
import math
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import pyzed.sl as sl
except ImportError:
    sys.exit("pyzed not found. Source the ZED SDK python API.")


VIEWS = ("stream", "mask", "verticals", "horizontals")


class _Frames:
    """One slot per view, last-write-wins. Encoding never blocks the detector."""

    def __init__(self):
        self.jpg = {v: None for v in VIEWS}
        self.lock = threading.Lock()
        self.clients = 0

    def put(self, view, jpg):
        with self.lock:
            self.jpg[view] = jpg

    def get(self, view):
        with self.lock:
            return self.jpg.get(view)


FRAME = _Frames()

PAGE = b"""<html><head><style>
body{margin:0;background:#111;color:#ccc;font:13px system-ui}
.g{display:grid;grid-template-columns:1fr 1fr;gap:6px;padding:6px}
figure{margin:0}img{width:100%;display:block;background:#000}
figcaption{padding:3px 2px;color:#8a8}
</style></head><body><div class="g">
<figure><img src="/stream"><figcaption>detections &mdash; green posts, orange bars, magenta gate bearing</figcaption></figure>
<figure><img src="/mask"><figcaption>white mask, after gap-bridging close &mdash; pipes should be SOLID</figcaption></figure>
<figure><img src="/verticals"><figcaption>merged vertical segments &mdash; green kept, red rejected by depth</figcaption></figure>
<figure><img src="/horizontals"><figcaption>merged horizontal segments &mdash; the crossbar</figcaption></figure>
</div></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass                       # silence per-request logging

    def do_GET(self):
        view = self.path.strip("/")
        if view not in VIEWS:
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(PAGE)
            return

        FRAME.clients += 1
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=f")
        self.end_headers()
        try:
            while True:
                jpg = FRAME.get(view)
                if jpg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " +
                                 str(len(jpg)).encode() + b"\r\n\r\n")
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(0.12)   # ~8 fps
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            FRAME.clients = max(0, FRAME.clients - 1)


class _Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class PipeDetector(Node):

    def __init__(self):
        super().__init__("pipe_detector")

        d = self.declare_parameter
        # PERFORMANCE is deprecated in recent SDKs and logs a warning; NEURAL
        # is the tuned GPU path on Jetson. Cap depth_maximum_distance instead
        # of reaching for a lighter mode.
        d("depth_mode", "NEURAL")
        d("resolution", "HD720")
        d("refraction_scale", 1.33)      # 1.0 when dry. MEASURE THIS.
        d("vision_range_max", 8.0)       # m, beyond this stereo is noise
        d("vision_range_min", 0.4)

        d("sat_max", 70)                 # HSV S ceiling for "white"
        d("val_min", 130)                # HSV V floor
        d("val_percentile", 88.0)        # adaptive floor, whichever is higher

        # gap bridging. close_len is in px and must exceed the largest dash
        # gap you see in the /mask view. Too large and background speckle
        # merges into ribbons -- watch the mask view when you raise it.
        d("close_len", 25)
        d("close_thick", 3)

        # Hough
        d("hough_thresh", 40)            # votes
        d("min_post_len_frac", 0.18)     # of image height
        d("min_bar_len_frac", 0.12)      # of image width
        d("max_line_gap", 25)            # px, stitches dashes
        d("angle_tol_deg", 15.0)         # +-tol about vertical / horizontal

        # collinear merge
        d("merge_angle_deg", 8.0)
        d("merge_dist_px", 18.0)

        # depth gate -- the phantom-killer
        d("require_depth", True)
        d("depth_valid_frac", 0.30)      # of samples along the segment
        d("depth_samples", 12)

        d("max_post_width_frac", 0.10)
        d("post_pair_tol_px", 40)        # crossbar end vs post centre
        d("gate_hold_s", 0.8)            # keep calling it a gate this long
        d("gate_hold_tol_px", 60)        # post drift allowed while holding
        d("pair_range_tol_m", 0.6)       # two posts at same depth -> gate
        d("vignette_frac", 0.0)          # 0 disables. 0.98 trims lens corners
        d("rate_hz", 10.0)
        d("show", False)
        d("mjpeg_port", 8080)            # 0 disables. http://<jetson>:8080
        d("mjpeg_width", 640)

        g = lambda n: self.get_parameter(n).value
        self.scale = float(g("refraction_scale"))
        self.rmax = float(g("vision_range_max"))
        self.rmin = float(g("vision_range_min"))
        self.sat_max = int(g("sat_max"))
        self.val_min = int(g("val_min"))
        self.val_pct = float(g("val_percentile"))
        self.close_len = int(g("close_len"))
        self.close_thick = int(g("close_thick"))
        self.hough_thresh = int(g("hough_thresh"))
        self.min_post_lf = float(g("min_post_len_frac"))
        self.min_bar_lf = float(g("min_bar_len_frac"))
        self.max_gap = int(g("max_line_gap"))
        self.ang_tol = float(g("angle_tol_deg"))
        self.merge_ang = float(g("merge_angle_deg"))
        self.merge_dist = float(g("merge_dist_px"))
        self.req_depth = bool(g("require_depth"))
        self.dvalid = float(g("depth_valid_frac"))
        self.dsamp = int(g("depth_samples"))
        self.max_pw = float(g("max_post_width_frac"))
        self.pair_tol = int(g("post_pair_tol_px"))
        self.hold_s = float(g("gate_hold_s"))
        self.hold_tol = int(g("gate_hold_tol_px"))
        self.pair_rtol = float(g("pair_range_tol_m"))
        self.vig = float(g("vignette_frac"))
        self.period = 1.0 / float(g("rate_hz"))
        self.show = bool(g("show"))
        self.mjpeg_port = int(g("mjpeg_port"))
        self.mjpeg_w = int(g("mjpeg_width"))

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

        self.min_post_len = self.min_post_lf * self.h
        self.min_bar_len = self.min_bar_lf * self.w

        self._roi = self._build_roi()

        self.get_logger().info(
            f"{self.w}x{self.h}  fx_air {fx_air:.1f} -> fx_eff {self.fx:.1f}  "
            f"scale {self.scale:.2f}  range_max {self.rmax:.1f}m  "
            f"depth_gate {'ON' if self.req_depth else 'OFF'}")

        self.img = sl.Mat()
        self.depth = sl.Mat()
        self.rt = sl.RuntimeParameters()

        self.pub = self.create_publisher(String, "/nautilus/detections", 10)

        if self.mjpeg_port:
            srv = _Server(("0.0.0.0", self.mjpeg_port), _Handler)
            threading.Thread(target=srv.serve_forever, daemon=True).start()
            self.get_logger().info(
                f"MJPEG on http://<jetson-ip>:{self.mjpeg_port}/  "
                f"(encodes only while a browser is attached)")

        # last confirmed gate: (post_x_left, post_x_right, crossbar_y, stamp)
        self._last_gate = None

    # ---------------- optics ----------------

    def bearing(self, x):
        """Pixel column -> degrees, + is right of centre. Pinhole, fx_eff."""
        return float(np.degrees(np.arctan2(x - self.w / 2.0, self.fx)))

    def range_at(self, dmap, x, y, half=6):
        """Median finite depth in a window, refraction-corrected.

        Returns (range_m, ok). ok=False means outside the trusted band, in
        which case the caller must not use the number.
        """
        x0, x1 = max(0, int(x) - half), min(self.w, int(x) + half)
        y0, y1 = max(0, int(y) - half), min(self.h, int(y) + half)
        patch = dmap[y0:y1, x0:x1]
        good = patch[np.isfinite(patch)]
        if good.size < 8:
            return 0.0, False
        z = float(np.median(good)) * self.scale
        return round(z, 2), bool(self.rmin <= z <= self.rmax)

    # ---------------- segmentation ----------------

    def _build_roi(self):
        """Static ellipse trimming lens-vignette corners. 0 disables."""
        if self.vig <= 0.0:
            return None
        roi = np.zeros((self.h, self.w), np.uint8)
        cv2.ellipse(roi, (self.w // 2, self.h // 2),
                    (int(self.w * self.vig / 2), int(self.h * self.vig / 2)),
                    0, 0, 360, 255, -1)
        return roi

    def white_mask(self, bgr):
        """Low saturation, high value, then bridge the dashes.

        The CLOSE runs twice with elongated kernels rather than once with a
        square one. A square kernel large enough to span a 25 px dash gap also
        blooms every speck of background noise into a blob; the anisotropic
        pair only bridges ALONG each pipe axis.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        s, v = hsv[:, :, 1], hsv[:, :, 2]
        floor = max(self.val_min, int(np.percentile(v, self.val_pct)))
        m = ((s < self.sat_max) & (v >= floor)).astype(np.uint8) * 255

        if self._roi is not None:
            m = cv2.bitwise_and(m, self._roi)

        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        t, L = max(1, self.close_thick), max(3, self.close_len)
        vk = cv2.getStructuringElement(cv2.MORPH_RECT, (t, L))
        hk = cv2.getStructuringElement(cv2.MORPH_RECT, (L, t))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, vk)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, hk)
        return m

    @staticmethod
    def _angle(seg):
        """Segment angle in [0,180)."""
        x1, y1, x2, y2 = seg
        return math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0

    @staticmethod
    def _length(seg):
        x1, y1, x2, y2 = seg
        return math.hypot(x2 - x1, y2 - y1)

    def _merge(self, segs):
        """Collapse collinear, overlapping fragments into one segment each.

        Hough returns several overlapping segments per physical pipe. Without
        this, the "two parallel verticals" test happily pairs two fragments of
        the SAME post and reports a gate of near-zero width -- a bearing that
        points straight at a single pipe.
        """
        merged = []
        for seg in sorted(segs, key=self._length, reverse=True):
            a = self._angle(seg)
            placed = False
            for grp in merged:
                da = abs(a - grp["ang"])
                da = min(da, 180.0 - da)
                if da > self.merge_ang:
                    continue
                # perpendicular distance from this segment's midpoint to the
                # group's infinite line
                mx, my = (seg[0] + seg[2]) / 2.0, (seg[1] + seg[3]) / 2.0
                gx, gy = grp["mid"]
                th = math.radians(grp["ang"])
                perp = abs(-(mx - gx) * math.sin(th) + (my - gy) * math.cos(th))
                if perp > self.merge_dist:
                    continue
                grp["pts"].extend([(seg[0], seg[1]), (seg[2], seg[3])])
                placed = True
                break
            if not placed:
                merged.append({
                    "ang": a,
                    "mid": ((seg[0] + seg[2]) / 2.0, (seg[1] + seg[3]) / 2.0),
                    "pts": [(seg[0], seg[1]), (seg[2], seg[3])],
                })

        out = []
        for grp in merged:
            pts = np.array(grp["pts"], dtype=np.float64)
            th = math.radians(grp["ang"])
            u = np.array([math.cos(th), math.sin(th)])
            proj = pts @ u
            p0 = pts[int(np.argmin(proj))]
            p1 = pts[int(np.argmax(proj))]
            out.append((float(p0[0]), float(p0[1]), float(p1[0]), float(p1[1])))
        return out

    def _depth_ok(self, dmap, seg):
        """Does this segment exist in 3D?

        Samples along the line and asks what fraction have finite disparity.
        A vignette corner, a glare patch, or a distant wall texture will be
        white in the mask and empty in the depth map. Returns (ok, frac).
        """
        if not self.req_depth:
            return True, 1.0
        x1, y1, x2, y2 = seg
        ts = np.linspace(0.15, 0.85, self.dsamp)   # skip the endpoints
        xs = np.clip((x1 + (x2 - x1) * ts).astype(int), 0, self.w - 1)
        ys = np.clip((y1 + (y2 - y1) * ts).astype(int), 0, self.h - 1)
        vals = dmap[ys, xs]
        frac = float(np.isfinite(vals).mean())
        return frac >= self.dvalid, frac

    def _to_comp(self, seg, idx):
        """Segment -> the dict shape classify() already expects."""
        x1, y1, x2, y2 = seg
        bx, by = int(min(x1, x2)), int(min(y1, y2))
        bw, bh = int(abs(x2 - x1)), int(abs(y2 - y1))
        return {
            "i": idx,
            "x": (x1 + x2) / 2.0,
            "y": (y1 + y2) / 2.0,
            "bx": bx, "by": by,
            "bw": max(bw, 1), "bh": max(bh, 1),
            "seg": seg,
            "len": self._length(seg),
        }

    def detect(self, mask, dmap):
        """mask -> (posts, bars, rejected). Rejected is for the debug view."""
        raw = cv2.HoughLinesP(mask, rho=1, theta=np.pi / 180,
                              threshold=self.hough_thresh,
                              minLineLength=int(min(self.min_post_len,
                                                    self.min_bar_len)),
                              maxLineGap=self.max_gap)
        if raw is None:
            return [], [], []

        segs = self._merge([tuple(s) for s in raw[:, 0]])

        posts, bars, rejected = [], [], []
        idx = 0
        for seg in segs:
            a = self._angle(seg)
            vertical = abs(a - 90.0) <= self.ang_tol
            horizontal = a <= self.ang_tol or a >= 180.0 - self.ang_tol
            if not (vertical or horizontal):
                continue

            L = self._length(seg)
            if vertical and L < self.min_post_len:
                continue
            if horizontal and L < self.min_bar_len:
                continue

            ok, _frac = self._depth_ok(dmap, seg)
            c = self._to_comp(seg, idx)
            idx += 1
            if not ok:
                rejected.append(c)
                continue
            if vertical:
                if c["bw"] <= self.max_pw * self.w:
                    posts.append(c)
            else:
                bars.append(c)

        posts.sort(key=lambda c: -c["len"])
        bars.sort(key=lambda c: -c["len"])
        return posts, bars, rejected

    # ---------------- classification ----------------

    def _emit_gate(self, a, b, crossbar_y, dmap, source):
        ra, oka = self.range_at(dmap, a["x"], a["y"])
        rb, okb = self.range_at(dmap, b["x"], b["y"])
        if oka and okb:
            rng, ok = (ra + rb) / 2.0, True
        elif oka or okb:
            rng, ok = (ra if oka else rb), True
        else:
            rng, ok = 0.0, False

        conf = min(1.0, (a["len"] + b["len"]) / (1.4 * self.h))
        if source != "crossbar":
            conf *= 0.7      # a held or range-inferred gate is less certain

        return {
            "bearing": round(self.bearing((a["x"] + b["x"]) / 2.0), 2),
            "range": round(rng, 2),
            "range_ok": bool(ok),
            "crossbar_y": crossbar_y,
            "posts": sorted([round(self.bearing(a["x"]), 2),
                             round(self.bearing(b["x"]), 2)]),
            "conf": round(conf, 2),
            "src": source,
        }

    def classify(self, posts, bars, dmap):
        """Identify the gate, then whatever vertical is left is the pole.

        Three ways to call something a gate, in descending order of trust:

          crossbar  a horizontal member spans two posts. Unambiguous.
          range     two posts at the same stereo depth. The pole is far
                    behind the gate, so equal depth means both are gate.
          hold      a gate was confirmed within gate_hold_s and the posts
                    have not moved far. Rides out a dropped crossbar frame.

        Without hysteresis the label flickers gate/pole at 10 Hz whenever the
        crossbar filter drops a frame, which is a real thing that happens.
        """
        now = time.time()
        out = {}
        gate_ids = set()

        # 1. crossbar. Its ends must land near two post centres, and it must
        #    sit near their TOPS -- a horizontal glare streak across the
        #    middle of two posts is not a crossbar.
        for bar in bars:
            lx, rx = bar["bx"], bar["bx"] + bar["bw"]
            L = [p for p in posts if abs(p["x"] - lx) < self.pair_tol]
            R = [p for p in posts if abs(p["x"] - rx) < self.pair_tol]
            if not (L and R):
                continue
            a = min(L, key=lambda p: abs(p["x"] - lx))
            b = min(R, key=lambda p: abs(p["x"] - rx))
            if a["i"] == b["i"]:
                continue
            top = min(a["by"], b["by"])
            span = max(a["bh"], b["bh"])
            if bar["y"] > top + 0.35 * span:
                continue
            if a["x"] > b["x"]:
                a, b = b, a
            gate_ids = {a["i"], b["i"]}
            cby = round(bar["y"] / self.h, 3)
            out["gate"] = self._emit_gate(a, b, cby, dmap, "crossbar")
            self._last_gate = (a["x"], b["x"], cby, now)
            break

        # 2. equal-depth pair
        if "gate" not in out and len(posts) >= 2:
            best = None
            for i in range(len(posts)):
                for j in range(i + 1, len(posts)):
                    a, b = posts[i], posts[j]
                    ra, oka = self.range_at(dmap, a["x"], a["y"])
                    rb, okb = self.range_at(dmap, b["x"], b["y"])
                    if not (oka and okb):
                        continue
                    if abs(ra - rb) <= self.pair_rtol:
                        score = a["len"] + b["len"]
                        if best is None or score > best[0]:
                            best = (score, a, b)
            if best:
                _, a, b = best
                if a["x"] > b["x"]:
                    a, b = b, a
                gate_ids = {a["i"], b["i"]}
                cby = self._last_gate[2] if self._last_gate else None
                out["gate"] = self._emit_gate(a, b, cby, dmap, "range")
                self._last_gate = (a["x"], b["x"], cby, now)

        # 3. hysteresis
        if "gate" not in out and self._last_gate and \
                (now - self._last_gate[3]) < self.hold_s:
            lx, rx, cby, _ = self._last_gate
            L = [p for p in posts if abs(p["x"] - lx) < self.hold_tol]
            R = [p for p in posts if abs(p["x"] - rx) < self.hold_tol]
            if L and R:
                a = min(L, key=lambda p: abs(p["x"] - lx))
                b = min(R, key=lambda p: abs(p["x"] - rx))
                if a["i"] != b["i"]:
                    if a["x"] > b["x"]:
                        a, b = b, a
                    gate_ids = {a["i"], b["i"]}
                    out["gate"] = self._emit_gate(a, b, cby, dmap, "hold")
                    # do NOT refresh the stamp: hold decays, never renews

        # pole = strongest leftover vertical
        leftover = [p for p in posts if p["i"] not in gate_ids]
        if leftover:
            p = max(leftover, key=lambda c: c["len"])
            rng, ok = self.range_at(dmap, p["x"], p["y"])
            out["pole"] = {
                "bearing": round(self.bearing(p["x"]), 2),
                "range": round(rng, 2),
                "range_ok": bool(ok),
                "conf": round(min(1.0, p["len"] / (0.6 * self.h)), 2),
            }

        out["n_posts"] = len(posts)
        out["n_bars"] = len(bars)
        return out

    # ---------------- debug rendering ----------------

    def _seg_view(self, comps, rejected, color):
        vis = np.zeros((self.h, self.w, 3), np.uint8)
        for c in rejected:
            x1, y1, x2, y2 = [int(v) for v in c["seg"]]
            cv2.line(vis, (x1, y1), (x2, y2), (0, 0, 220), 2)
        for c in comps:
            x1, y1, x2, y2 = [int(v) for v in c["seg"]]
            cv2.line(vis, (x1, y1), (x2, y2), color, 3)
        return vis

    def annotate(self, bgr, posts, bars, rejected, det):
        vis = bgr.copy()
        for c in rejected:
            x1, y1, x2, y2 = [int(v) for v in c["seg"]]
            cv2.line(vis, (x1, y1), (x2, y2), (0, 0, 220), 1)
        for p in posts:
            cv2.rectangle(vis, (p["bx"], p["by"]),
                          (p["bx"] + p["bw"], p["by"] + p["bh"]),
                          (0, 255, 0), 2)
        for b in bars:
            cv2.rectangle(vis, (b["bx"], b["by"]),
                          (b["bx"] + b["bw"], b["by"] + b["bh"]),
                          (0, 160, 255), 2)
        if "gate" in det:
            g = det["gate"]
            cx = int(self.w / 2 + self.fx * np.tan(np.radians(g["bearing"])))
            cv2.line(vis, (cx, 0), (cx, self.h), (255, 80, 255), 2)
            cv2.putText(vis, f"GATE b{g['bearing']:+.1f} r{g['range']:.2f} "
                             f"{g['src']} c{g['conf']:.2f}",
                        (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 80, 255), 2)
        if "pole" in det:
            p = det["pole"]
            cv2.putText(vis, f"POLE b{p['bearing']:+.1f} "
                             f"r{p['range'] if p['range_ok'] else '--'}",
                        (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (80, 220, 255), 2)
        cv2.putText(vis, f"posts {det['n_posts']}  bars {det['n_bars']}  "
                         f"rej {len(rejected)}",
                    (8, self.h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)
        cv2.line(vis, (self.w // 2, 0), (self.w // 2, self.h), (90, 90, 90), 1)
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

            mask = self.white_mask(bgr)
            posts, bars, rejected = self.detect(mask, dmap)
            det = self.classify(posts, bars, dmap)
            det["t"] = round(time.time(), 3)

            m = String()
            m.data = json.dumps(det)
            self.pub.publish(m)

            # Encode only when someone is watching. A detached browser costs
            # nothing; this runs on the same Orin as the depth engine.
            if self.mjpeg_port and FRAME.clients > 0:
                sc = self.mjpeg_w / float(self.w)
                dim = (self.mjpeg_w, int(self.h * sc))
                jq = [cv2.IMWRITE_JPEG_QUALITY, 70]

                for view, im in (
                        ("stream", self.annotate(bgr, posts, bars, rejected, det)),
                        ("mask", mask),
                        ("verticals", self._seg_view(posts, rejected, (0, 255, 0))),
                        ("horizontals", self._seg_view(bars, [], (0, 160, 255)))):
                    small = cv2.resize(im, dim, interpolation=cv2.INTER_NEAREST)
                    okj, buf = cv2.imencode(".jpg", small, jq)
                    if okj:
                        FRAME.put(view, buf.tobytes())

            if self.show:
                cv2.imshow("detections",
                           self.annotate(bgr, posts, bars, rejected, det))
                cv2.imshow("white mask", mask)
                cv2.imshow("verticals",
                           self._seg_view(posts, rejected, (0, 255, 0)))
                if cv2.waitKey(1) == 27:
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
