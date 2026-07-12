#!/usr/bin/env python3
"""
tracking.py - Generic confirm-streak state tracker.

Turns a stream of noisy, intermittent detections into a stable "confirmed"
state: the first sighting is taken immediately (there's no value sitting
blind with zero prior), and after that the held state only moves once a run
of consecutive detections agree with each other on some scalar field (e.g.
range) within an epsilon. A single noisy reading, or a missed frame, does
not yank the state around -- a miss just holds the last confirmed value and
resets the in-progress agreement streak.

No cv2/ROS/camera dependency: usable from a plain OpenCV script, a ROS2
node, or a unit test feeding it fake dicts.
"""


class Tracker:

    def __init__(self, key, confirm_frames=3, epsilon=0.3):
        self.key = key
        self.confirm_frames = confirm_frames
        self.epsilon = epsilon
        self.confirmed = None      # last confirmed detection dict, or None
        self.streak = 0
        self._streak_value = None

    def update(self, detection):
        """detection: a dict containing self.key, or None on a miss.
        Returns the current confirmed detection (or None if never seen)."""
        if detection is None or detection.get(self.key) is None:
            self.streak = 0
            self._streak_value = None
            return self.confirmed

        v = detection[self.key]
        if self._streak_value is not None and abs(v - self._streak_value) <= self.epsilon:
            self.streak += 1
        else:
            self.streak = 1
            self._streak_value = v

        if self.confirmed is None or self.streak >= self.confirm_frames:
            self.confirmed = detection
        return self.confirmed
