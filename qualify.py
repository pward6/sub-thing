"""
Look for the gate -> descend for a specified time -> drive forward for
5 seconds -> stop -> disarm

The vehicle switches to STABILIZE immediately after arming. SEEK_ALTITUDE
retains its existing function name for compatibility, but it no longer
reads the DVL or altimeter and does not calculate depth or altitude.

While in STABILIZE, SEEK_ALTITUDE commands a constant vertical thrust for
ALTITUDE_TIMEOUT seconds. After the timed descent is complete, the vehicle
returns its vertical command to neutral and drives forward for
FORWARD_DRIVE_TIME seconds.

After the forward movement is complete, all movement commands are returned
to neutral and the vehicle is disarmed.

ALTITUDE_SIGN controls the direction of the vertical command. With the
current convention, ALTITUDE_SIGN = 1.0 commands a z value below
Z_NEUTRAL. If the vehicle rises instead of descending, change
ALTITUDE_SIGN to -1.0.
"""


# SEEK_ALTITUDE
#
# Existing variable names are retained so other parts of the program do not
# break. SEEK_ALTITUDE is now a timed descent and does not use DVL or
# altimeter measurements.
TARGET_ALTITUDE_M = 0.0           # retained for compatibility; no longer used
ALTITUDE_TOLERANCE_M = 0.1        # retained for compatibility; no longer used
ALTITUDE_KP = 300.0               # retained for compatibility; no longer used

ALTITUDE_Z_MAX = 200.0            # fixed z offset from neutral during descent
ALTITUDE_TIMEOUT = 5.0            # seconds to descend

ALTIMETER_TIMEOUT = 3.0           # retained for compatibility; no longer used here
ALTITUDE_MIN_SAFE_M = 0.0         # retained for compatibility; no longer used here
ALTITUDE_SIGN = 1.0               # change to -1.0 if the vehicle rises
ALTITUDE_STALL_CHECK_S = 8.0      # retained for compatibility; no longer used
ALTITUDE_STALL_MIN_MOVE_M = 0.03  # retained for compatibility; no longer used

# Forward-drive settings
FORWARD_DRIVE_TIME = 5.0          # seconds to drive forward after descending
FORWARD_DRIVE_X = 200.0           # forward command; increase or decrease as needed


# Fixed tuning constants. These get set once from pool testing and rarely
# change between runs -- edit them here rather than adding another ROS
# parameter.


def __init__(self):
    d = self.declare_parameter

    d("dry_run", False)

    d(
        "fake_gate",
        True
    )  # bench default; pass fake_gate:=false for a real run

    d("fake_gate_bearing_deg", 0.0)
    d("fake_gate_range_m", 3.0)
    d("target_heading", float("nan"))

    # Keep the remainder of the existing __init__ function unchanged.


def run(self):
    # Keep all existing code above this section unchanged.

    if not self.arm(True):
        self._log("fatal", "arm rejected (SYSID_MYGCS=1?)")
        return 1

    # STABILIZE provides direct vertical and forward control while still
    # stabilizing roll and pitch.
    if not self.set_mode("STABILIZE"):
        self._log("fatal", "mode rejected")
        self.arm(False)
        return 1

    try:
        # Timed descent followed by timed forward movement.
        self.seek_altitude()

    except Abort as exc:
        self._log("fatal", str(exc))

        # Stop all movement before disarming after an abort.
        self.publish(
            0.0,
            0.0,
            z=Z_NEUTRAL
        )

        self.arm(False)
        return 1

    # Stop all movement before disarming.
    self.publish(
        0.0,
        0.0,
        z=Z_NEUTRAL
    )

    self._log("info", "mission complete. Disarming.")

    if not self.arm(False):
        self._log("fatal", "disarm rejected")
        return 1

    return 0


def seek_altitude(self):
    """
    Descend for a fixed amount of time without using DVL or altimeter data,
    then drive forward for an additional fixed amount of time.

    The existing function name is retained so existing calls to
    seek_altitude() do not need to change.

    During the descent:

    - The vehicle remains in STABILIZE.
    - Forward thrust stays at zero.
    - Yaw continues holding gate_heading.
    - DVL and altimeter readings are ignored.
    - A fixed vertical command is applied for ALTITUDE_TIMEOUT seconds.

    After the descent:

    - Vertical thrust returns to neutral.
    - The vehicle remains in STABILIZE.
    - The vehicle drives forward for FORWARD_DRIVE_TIME seconds.
    - Yaw continues holding gate_heading.
    - The vehicle stops and returns control to run(), which disarms it.
    """
    self.enter("SEEK_ALTITUDE")

    # -------------------------------------------------------------
    # Timed descent
    # -------------------------------------------------------------

    descent_end = time.time() + ALTITUDE_TIMEOUT

    # With the default values:
    #
    # Z_NEUTRAL = 500
    # ALTITUDE_SIGN = 1.0
    # ALTITUDE_Z_MAX = 200
    #
    # z = 500 - (1.0 * 200) = 300
    descent_z = clamp(
        Z_NEUTRAL - ALTITUDE_SIGN * ALTITUDE_Z_MAX,
        0.0,
        1000.0
    )

    while True:
        now = time.time()

        if now >= descent_end:
            break

        remaining = max(0.0, descent_end - now)

        self.log_every(
            "seek_altitude",
            1.0,
            lambda: (
                f"  timed descent  "
                f"{remaining:.1f}s remaining  "
                f"z {descent_z:.0f} "
                f"(neutral {Z_NEUTRAL:.0f}, "
                f"delta {descent_z - Z_NEUTRAL:+.0f})"
            )
        )

        # No forward movement during descent.
        # Continue holding the gate heading.
        # Ignore all DVL and altimeter readings.
        self.tick(
            0.0,
            self.yaw_to(self.gate_heading),
            z=descent_z
        )

    # Return vertical thrust to neutral before driving forward.
    self.publish(
        0.0,
        self.yaw_to(self.gate_heading),
        z=Z_NEUTRAL
    )

    self._log(
        "info",
        f"timed descent complete after {ALTITUDE_TIMEOUT:.1f}s. "
        f"Driving forward for {FORWARD_DRIVE_TIME:.1f}s."
    )

    # -------------------------------------------------------------
    # Timed forward movement
    # -------------------------------------------------------------

    forward_end = time.time() + FORWARD_DRIVE_TIME

    while True:
        now = time.time()

        if now >= forward_end:
            break

        remaining = max(0.0, forward_end - now)

        self.log_every(
            "timed_forward_drive",
            1.0,
            lambda: (
                f"  timed forward drive  "
                f"{remaining:.1f}s remaining  "
                f"x {FORWARD_DRIVE_X:.0f}"
            )
        )

        # Drive forward while keeping vertical thrust neutral and
        # continuing to hold the gate heading.
        self.tick(
            FORWARD_DRIVE_X,
            self.yaw_to(self.gate_heading),
            z=Z_NEUTRAL
        )

    # Stop forward, yaw, and vertical movement.
    self.publish(
        0.0,
        0.0,
        z=Z_NEUTRAL
    )

    self._log(
        "info",
        f"forward drive complete after {FORWARD_DRIVE_TIME:.1f}s. "
        "Stopping before disarm."
    )


def settle(self):
    """Hold heading, no forward thrust. There is no depth logic here:"""

    # Keep the rest of the existing settle() function unchanged.
