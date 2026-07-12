#!/usr/bin/env python3
"""
gate_shape.py - Camera-agnostic pixel geometry for the PVC gate and pole.

Everything here works on a plain grayscale image and knows nothing about
lenses, stereo depth, or bearing math -- that's the caller's job (see
gate_detector.py for a pinhole-based version, pipe_detector.py for a ZED
stereo-calibrated version). This module only answers "where, in pixels, is
the gate/pole in this image?"

ALGORITHM
    An HSV "white pipe" colour mask does not work on real underwater
    footage: the water's blue cast pushes saturation up almost everywhere,
    including on the pipe, so "low saturation" stops separating pipe from
    water. What actually distinguishes the pipe is that it's a thin ridge
    slightly brighter than its immediate surroundings -- often only barely
    so, once it's far from the camera or in a dim corner. So instead:

      1. find the CROSSBAR first (Canny edges -> Hough -> merge). It's
         near-horizontal, spans a good chunk of the frame, and is usually
         the highest-contrast part of the gate -- the easiest anchor.
      2. walk outward from each of the crossbar's ends, column by column,
         to find where it truly terminates (Hough/Canny often only catch
         the well-lit middle section).
      3. from each true corner, walk DOWN, row by row, following whichever
         nearby pixel is brighter than a smoothed local background -- not a
         fixed threshold. This recovers a post so faint that no edge or
         colour mask would ever fire on it.

    The corner where crossbar meets post is a PVC fitting -- a blob wider
    than the pipe -- so each walk skips a few pixels past its start before
    trusting what it finds, and searches a narrow, slope-predicted window
    after that, so it can't leap onto an unrelated bright pixel (a lane
    rope, a shadow line).

    A crossbar with two legs that each reach a decent fraction of the frame
    height IS the three-sided gate, by construction: there was never a
    search for a bottom bar.

    The pole is whatever else in frame is a strong, isolated near-vertical
    edge, once anything overlapping the gate's own legs is excluded.
"""

import cv2
import numpy as np

# ---- tuning knobs -----------------------------------------------------
CANNY_LO, CANNY_HI = 30, 90
VALID_BRIGHTNESS_MIN = 25     # below this: lens vignette / letterbox, not scene

HOUGH_THRESHOLD = 60
MIN_CROSSBAR_LEN_FRAC = 0.15   # of frame width
MAX_LINE_GAP = 25
HORIZONTAL_TOL_DEG = 12.0
VERTICAL_TOL_DEG = 15.0

MERGE_ANGLE_DEG = 8.0
MERGE_DIST_PX = 18.0

BG_BLUR_SIGMA = 41            # smoothed-background scale for ridge tracing
RIDGE_BAND_PX = 10            # half-width of the search window, in px
RIDGE_SKIP_PX = 12            # px to skip past a corner fitting before tracing
RIDGE_GAP_TOL = 15            # consecutive misses tolerated before giving up
RIDGE_THRESH = 3.0            # residual brightness a ridge pixel must clear
RIDGE_SLOPE_SMOOTH = 0.7      # inertia on the predicted walk direction

MIN_LEG_LEN_FRAC = 0.15       # of frame height; shorter -> not a confirmed leg
MIN_POLE_LEN_FRAC = 0.15      # of frame height


def background_residual(gray):
    """Smoothed-background-subtracted image used by every ridge walk."""
    gray_f = gray.astype(np.float32)
    return gray_f - cv2.GaussianBlur(gray_f, (0, 0), sigmaX=BG_BLUR_SIGMA)


def _angle(seg):
    x1, y1, x2, y2 = seg
    return float(np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180.0)


def _length(seg):
    x1, y1, x2, y2 = seg
    return float(np.hypot(x2 - x1, y2 - y1))


def _valid_region(gray):
    """Mask out lens-vignette corners / letterbox bars: anything too dark
    to be real scene content, so it can't masquerade as a long straight
    edge."""
    return cv2.erode((gray > VALID_BRIGHTNESS_MIN).astype(np.uint8) * 255,
                     np.ones((5, 5), np.uint8))


def _edges(gray):
    e = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), CANNY_LO, CANNY_HI)
    return cv2.bitwise_and(e, _valid_region(gray))


def merge_collinear(segs):
    """Collapse Hough's overlapping fragments into one segment per edge."""
    groups = []
    for seg in sorted(segs, key=_length, reverse=True):
        a = _angle(seg)
        mx, my = (seg[0] + seg[2]) / 2.0, (seg[1] + seg[3]) / 2.0
        placed = False
        for g in groups:
            da = abs(a - g["ang"])
            da = min(da, 180.0 - da)
            if da > MERGE_ANGLE_DEG:
                continue
            th = np.radians(g["ang"])
            gx, gy = g["mid"]
            perp = abs(-(mx - gx) * np.sin(th) + (my - gy) * np.cos(th))
            if perp > MERGE_DIST_PX:
                continue
            g["pts"].extend([(seg[0], seg[1]), (seg[2], seg[3])])
            placed = True
            break
        if not placed:
            groups.append({"ang": a, "mid": (mx, my),
                           "pts": [(seg[0], seg[1]), (seg[2], seg[3])]})

    out = []
    for g in groups:
        pts = np.array(g["pts"], dtype=np.float64)
        th = np.radians(g["ang"])
        u = np.array([np.cos(th), np.sin(th)])
        proj = pts @ u
        p0, p1 = pts[np.argmin(proj)], pts[np.argmax(proj)]
        out.append((float(p0[0]), float(p0[1]), float(p1[0]), float(p1[1])))
    return out


def find_crossbars(gray):
    """gray -> near-horizontal merged segments, longest first."""
    w = gray.shape[1]
    raw = cv2.HoughLinesP(_edges(gray), rho=1, theta=np.pi / 180,
                          threshold=HOUGH_THRESHOLD,
                          minLineLength=int(MIN_CROSSBAR_LEN_FRAC * w),
                          maxLineGap=MAX_LINE_GAP)
    if raw is None:
        return []
    segs = merge_collinear([tuple(s) for s in raw.reshape(-1, 4)])
    bars = [s for s in segs if _angle(s) < HORIZONTAL_TOL_DEG
           or _angle(s) > 180.0 - HORIZONTAL_TOL_DEG]
    bars.sort(key=_length, reverse=True)
    return bars


def find_verticals(gray):
    """gray -> near-vertical merged segments, longest first."""
    h = gray.shape[0]
    raw = cv2.HoughLinesP(_edges(gray), rho=1, theta=np.pi / 180,
                          threshold=HOUGH_THRESHOLD,
                          minLineLength=int(MIN_POLE_LEN_FRAC * h),
                          maxLineGap=MAX_LINE_GAP)
    if raw is None:
        return []
    segs = merge_collinear([tuple(s) for s in raw.reshape(-1, 4)])
    verts = [s for s in segs if abs(_angle(s) - 90.0) < VERTICAL_TOL_DEG]
    verts.sort(key=_length, reverse=True)
    return verts


def _walk(residual, x0, y0, dx, dy, span, skip):
    """Follow a faint ridge one step (dx,dy) at a time from (x0,y0).

    Generic in direction so the same walk serves both the sideways
    crossbar-extension and the downward leg-trace: `dx,dy` is the unit step
    ((+-1, 0) or (0, +-1)), `span` bounds how far to walk.
    """
    h, w = residual.shape
    lat = 0.0            # position perpendicular to the walk direction
    slope = 0.0           # predicted drift per step
    pts = [(int(x0), int(y0))]
    miss = 0
    for i in range(skip, span):
        px, py = x0 + dx * i, y0 + dy * i
        pred = lat + slope
        if dx:   # walking horizontally -> search a vertical window
            lo, hi = int(py + pred) - RIDGE_BAND_PX, int(py + pred) + RIDGE_BAND_PX
            lo, hi = max(0, lo), min(h, hi)
            if int(px) < 0 or int(px) >= w or hi <= lo:
                break
            window = residual[lo:hi, int(px)]
        else:    # walking vertically -> search a horizontal window
            lo, hi = int(px + pred) - RIDGE_BAND_PX, int(px + pred) + RIDGE_BAND_PX
            lo, hi = max(0, lo), min(w, hi)
            if int(py) < 0 or int(py) >= h or hi <= lo:
                break
            window = residual[int(py), lo:hi]

        j = int(np.argmax(window))
        peak = window[j]
        if peak > RIDGE_THRESH:
            new_lat = (lo + j) - (py if dx else px)
            slope = RIDGE_SLOPE_SMOOTH * slope + (1 - RIDGE_SLOPE_SMOOTH) * (new_lat - lat)
            lat = new_lat
            pt = (int(px), int(lo + j)) if dx else (int(lo + j), int(py))
            pts.append(pt)
            miss = 0
        else:
            lat = pred
            miss += 1
            if miss > RIDGE_GAP_TOL:
                break
    return pts


def extend_crossbar(residual, end, direction, max_w):
    x0, y0 = end
    return _walk(residual, x0, y0, dx=direction, dy=0, span=max_w, skip=RIDGE_SKIP_PX)


def trace_leg(residual, corner, max_h):
    x0, y0 = corner
    pts = _walk(residual, x0, y0, dx=0, dy=1, span=max_h, skip=RIDGE_SKIP_PX)
    length = pts[-1][1] - y0
    return pts, length


def find_gate_shape(gray, residual):
    """-> dict{crossbar, left_leg, right_leg, left_len, right_len, conf} or
    None. Pixel-space only: no bearing, no range."""
    h, w = gray.shape

    for bar in find_crossbars(gray):
        x1, y1, x2, y2 = bar
        left, right = ((x1, y1), (x2, y2)) if x1 < x2 else ((x2, y2), (x1, y1))

        left_ext = extend_crossbar(residual, left, direction=-1, max_w=int(0.4 * w))
        right_ext = extend_crossbar(residual, right, direction=1, max_w=int(0.4 * w))
        true_left, true_right = left_ext[-1], right_ext[-1]

        left_leg, left_len = trace_leg(residual, true_left, max_h=int(h * 0.7))
        right_leg, right_len = trace_leg(residual, true_right, max_h=int(h * 0.7))

        min_leg = MIN_LEG_LEN_FRAC * h
        if left_len < min_leg or right_len < min_leg:
            continue     # this "crossbar" doesn't hang over two real legs

        span_frac = abs(true_right[0] - true_left[0]) / w
        leg_frac = min(left_len, right_len) / (0.6 * h)
        conf = round(min(1.0, span_frac) * min(1.0, leg_frac), 2)

        return {
            "crossbar": (true_left[0], true_left[1], true_right[0], true_right[1]),
            "left_leg": left_leg,
            "right_leg": right_leg,
            "left_len": left_len,
            "right_len": right_len,
            "conf": conf,
        }
    return None


def find_pole_shape(gray, exclude_x_ranges=()):
    """-> dict{segment, length, conf} or None.

    `exclude_x_ranges`: iterable of (x_lo, x_hi) columns to ignore, so the
    gate's own legs (already found) can't be re-reported as the pole.
    """
    h = gray.shape[0]
    for seg in find_verticals(gray):
        cx = (seg[0] + seg[2]) / 2.0
        if any(lo <= cx <= hi for lo, hi in exclude_x_ranges):
            continue
        length = _length(seg)
        if length < MIN_POLE_LEN_FRAC * h:
            continue
        return {"segment": seg, "length": length,
               "conf": round(min(1.0, length / (0.6 * h)), 2)}
    return None
