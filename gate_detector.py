#!/usr/bin/env python3
"""
gate_detector.py - Standalone PVC gate detector for any camera feed.

The gate-finding algorithm itself lives in gate_shape.py (camera-agnostic
pixel geometry, shared with the ROS/ZED node pipe_detector.py). This file
adds the piece that's specific to an arbitrary webcam/video: a pinhole
bearing/range estimate from a guessed field of view and a known gate width,
since there's no camera calibration or stereo depth to fall back on here.

    python3 gate_detector.py                # webcam 0
    python3 gate_detector.py path/to.mp4     # video file
    python3 gate_detector.py 1               # webcam index 1
    python3 gate_detector.py image.png       # single image, saves *_detected.png
    python3 gate_detector.py path/to.mp4 --tracked   # smoothed via GateTracker
"""

import sys

import cv2
import numpy as np

import gate_shape as shape
from tracking import Tracker

GATE_WIDTH_M = 2.0            # post-to-post, outer measurement of the real gate
CAMERA_FOV_DEG = 100.0        # horizontal field of view of the camera

CONFIRM_FRAMES = 3     # consecutive agreeing frames before the estimate moves
DISTANCE_EPSILON_M = 0.3   # "agreeing" means within this many metres


def focal_px(frame_w):
    """Pinhole focal length in pixels, derived from the horizontal FOV."""
    return (frame_w / 2.0) / np.tan(np.radians(CAMERA_FOV_DEG / 2.0))


def bearing_deg(x, frame_w):
    """Pixel column -> degrees off boresight, + is right of centre."""
    return float(np.degrees(np.arctan2(x - frame_w / 2.0, focal_px(frame_w))))


def distance_m(pixel_span, frame_w):
    """How far away a GATE_WIDTH_M-wide object must be to span pixel_span px.

    Similar-triangles pinhole estimate: real_width / distance = pixel_span / f_px.
    Assumes the gate is being viewed roughly head-on -- a gate seen at a steep
    angle will foreshorten and this will underestimate range.
    """
    if pixel_span <= 0:
        return None
    return GATE_WIDTH_M * focal_px(frame_w) / pixel_span


def find_gate(frame):
    """frame (BGR) -> dict describing the gate, or None.

    {"crossbar": (x1,y1,x2,y2), "left_leg": [...], "right_leg": [...],
     "conf": 0..1, "offset": -1..+1, "bearing_deg": ..., "distance_m": ...}
    offset is the gate centre's horizontal position, normalised so 0 is
    frame-centre. bearing_deg/distance_m come from GATE_WIDTH_M and
    CAMERA_FOV_DEG above -- set them for your camera and your gate before
    trusting the number.
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gate = shape.find_gate_shape(gray, shape.background_residual(gray))
    if gate is None:
        return None

    x1, y1, x2, y2 = gate["crossbar"]
    cx = (x1 + x2) / 2.0
    span = float(np.hypot(x2 - x1, y2 - y1))
    dist = distance_m(span, w)

    return {
        "crossbar": gate["crossbar"],
        "left_leg": gate["left_leg"],
        "right_leg": gate["right_leg"],
        "conf": gate["conf"],
        "offset": round((cx - w / 2.0) / (w / 2.0), 3),
        "bearing_deg": round(bearing_deg(cx, w), 2),
        "distance_m": round(dist, 2) if dist else None,
    }


class GateTracker(Tracker):
    """Tracker specialised for find_gate(): feed it frames, not detections."""

    def __init__(self, confirm_frames=CONFIRM_FRAMES, epsilon_m=DISTANCE_EPSILON_M):
        super().__init__(key="distance_m", confirm_frames=confirm_frames, epsilon=epsilon_m)

    def update(self, frame):
        return super().update(find_gate(frame))


def annotate(frame, gate):
    vis = frame.copy()
    if not gate:
        cv2.putText(vis, "no gate", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                   (0, 0, 255), 2)
        return vis

    x1, y1, x2, y2 = gate["crossbar"]
    cv2.line(vis, (x1, y1), (x2, y2), (255, 80, 255), 3)
    for leg in (gate["left_leg"], gate["right_leg"]):
        for i in range(1, len(leg)):
            cv2.line(vis, leg[i - 1], leg[i], (0, 255, 0), 3)

    cx = int((x1 + x2) / 2)
    dist = f"{gate['distance_m']:.2f}m" if gate["distance_m"] else "--"
    cv2.putText(vis, f"GATE conf {gate['conf']:.2f}  bearing {gate['bearing_deg']:+.1f}  "
                    f"dist {dist}",
               (cx - 180, y1 - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 80, 255), 2)
    return vis


def main():
    source = sys.argv[1] if len(sys.argv) > 1 else 0
    if isinstance(source, str) and source.isdigit():
        source = int(source)

    if isinstance(source, str) and source.lower().endswith((".png", ".jpg", ".jpeg")):
        frame = cv2.imread(source)
        if frame is None:
            sys.exit(f"could not read image: {source!r}")
        gate = find_gate(frame)
        print(gate if gate else "no gate found")
        out = source.rsplit(".", 1)[0] + "_detected.png"
        cv2.imwrite(out, annotate(frame, gate))
        print(f"wrote {out}")
        return

    tracked = "--tracked" in sys.argv
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        sys.exit(f"could not open camera/video source: {source!r}")

    tracker = GateTracker()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gate = tracker.update(frame) if tracked else find_gate(frame)
        vis = annotate(frame, gate)
        if tracked:
            cv2.putText(vis, f"streak {tracker.streak}/{tracker.confirm_frames}",
                       (20, vis.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                       (0, 255, 255), 2)
        cv2.imshow("gate_detector  (q to quit)", vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
