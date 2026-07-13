
   Look for the gate -> drive to it -> stop -> disarm

ALMOST NO DEPTH LOGIC. Mode is ALT_HOLD: ArduSub regulates depth on its own
barometer and holds whatever depth the vehicle is at. The ONE deliberate
exception is SEEK_ALTITUDE, right after arming: it reads
/nucleus_node/altimeter_packets and drives z until the vehicle is
TARGET_ALTITUDE_M above the pool floor, then hands neutral z straight back
to ALT_HOLD. Every other phase never touches z.
ALMOST NO DEPTH LOGIC. Mode is ALT_HOLD everywhere except the ONE
deliberate exception, SEEK_ALTITUDE, right after arming: ALT_HOLD's z
channel is a capped climb/descent RATE around a hold point (the mode's job
is to resist depth change), too gentle for actively driving to a target
depth. So SEEK_ALTITUDE switches to STABILIZE (direct z response, still
levels roll/pitch), reads /nucleus_node/altimeter_packets and drives z
until the vehicle is TARGET_ALTITUDE_M above the pool floor, then switches
back to ALT_HOLD before anything else runs. Every other phase never
touches z, and ALT_HOLD does the actual holding for the rest of the run.

SEEK_ALTITUDE's z-direction (ALTITUDE_SIGN) is UNVERIFIED -- whether raising
z moves the vehicle up or down is an ArduSub/wiring convention this code
@@ -71,15 +74,19 @@
# SEEK_ALTITUDE (the one exception to "no depth logic" -- see module docstring)
TARGET_ALTITUDE_M = 0.0      # m above the pool floor to reach before SETTLE
ALTITUDE_TOLERANCE_M = 0.1    # m: within this band of target counts as "there"
ALTITUDE_KP = 150.0           # z units per metre of error -- gentle; verify in pool
ALTITUDE_Z_MAX = 80.0         # z units off neutral, hard cap (of the 0-1000 range)
ALTITUDE_KP = 300.0           # z units per metre of error -- verify in pool
ALTITUDE_Z_MAX = 200.0        # z units off neutral, hard cap (of the 0-1000 range) --
                              # raised from 80: that wasn't moving the vehicle fast
                              # enough. Still short of full-scale (500) on purpose.
ALTITUDE_TIMEOUT = 120.0      # s to reach target before aborting
ALTIMETER_TIMEOUT = 3.0       # s: altimeter reports in pulses, not continuously --
# this must be looser than INS_TIMEOUT or every gap
# between pulses reads as "stale" and the z command
# pulses on/off in lockstep with the sensor.
ALTITUDE_MIN_SAFE_M = 0.0  # m: abort immediately this close to the floor, no matter what
ALTITUDE_SIGN = 1.0           # UNVERIFIED direction -- flip to -1.0 if it moves the wrong way
ALTITUDE_STALL_CHECK_S = 8.0   # s: window to judge "is it actually moving"
ALTITUDE_STALL_MIN_MOVE_M = 0.03   # m: less than this over the window counts as stalled

# Fixed tuning constants. These get set once from pool testing and rarely
# change between runs -- edit them here rather than adding another ROS
@@ -117,7 +124,8 @@ def __init__(self):

d = self.declare_parameter
d("dry_run", False)
        d("fake_gate", False)               # bench: fake a solid gate, no camera needed
        d("fake_gate", True)                # bench: fake a solid gate, no camera needed --
                                             # default ON; pass fake_gate:=false for a real run
d("fake_gate_bearing_deg", 0.0)     # bearing to report while faking
d("fake_gate_range_m", 3.0)         # range simulated CLOSING to by the end of the window
d("target_heading", float("nan"))   # NaN -> capture at arm
@@ -579,7 +587,14 @@ def run(self):
if not self.arm(True):
self._log("fatal", "arm rejected (SYSID_MYGCS=1?)")
return 1
        if not self.set_mode("ALT_HOLD"):
        # STABILIZE for SEEK_ALTITUDE: ALT_HOLD's z channel is a capped
        # climb/descent RATE around a hold point (the mode's whole job is
        # to resist depth change) -- too gentle for actively driving to a
        # target depth. STABILIZE gives z a much more direct thruster
        # response while still levelling roll/pitch. seek_altitude()
        # switches back to ALT_HOLD itself once it reaches target, before
        # anything else runs.
        if not self.set_mode("STABILIZE"):
self._log("fatal", "mode rejected")
self.arm(False)
return 1
@@ -690,20 +705,30 @@ def seek_altitude(self):
       ALTITUDE_MIN_SAFE_M aborts immediately regardless of that -- if the
       floor gets too close for any reason, stop before asking questions.

        x and r are pinned to 0.0 for the entire phase -- no forward thrust,
        no yaw correction, ever. This phase only ever commands z.
        x (forward thrust) is pinned to 0.0 for the entire phase -- it never
        drives. r (yaw) DOES stay active, holding gate_heading throughout,
        so a disturbance can't spin the vehicle off heading during the up
        to ALTITUDE_TIMEOUT seconds this phase may take.

       A stale altimeter reading does NOT abort the run: it holds neutral
       z (never drives on a number that might be old) and waits for a
       fresh one. Only ALTITUDE_TIMEOUT overall, or the safety floor, can
       end this phase early.

        Whether ALTITUDE_Z_MAX/ALTITUDE_KP actually produce enough thrust to
        move the vehicle is its own unverified thing -- this tracks the
        altimeter's real rate of change against a reference point every
        ALTITUDE_STALL_CHECK_S seconds, and if it hasn't moved at least
        ALTITUDE_STALL_MIN_MOVE_M despite commanding non-neutral z, logs a
        clear warning instead of silently grinding away with no effect.
       """
self.enter("SEEK_ALTITUDE")
if not self.spin_until(lambda: self.altimeter_distance is not None,
10.0, "altimeter"):
raise Abort("no altimeter data -- cannot seek altitude")

end = time.time() + ALTITUDE_TIMEOUT
        stall_ref_alt, stall_ref_time = None, None
while True:
if time.time() > end:
raise Abort(f"could not reach {TARGET_ALTITUDE_M:.2f}m altitude in "
@@ -715,7 +740,7 @@ def seek_altitude(self):
self.log_every("seek_altitude_stale", 1.0, lambda: (
f"  altimeter stale ({age:.2f}s old, last reading {alt:.2f}m) "
f"-- holding, waiting for a fresh one"))
                self.tick(0.0, 0.0, z=Z_NEUTRAL)   # never act on a possibly-stale number
                self.tick(0.0, self.yaw_to(self.gate_heading), z=Z_NEUTRAL)   # never act on a possibly-stale number
continue

if alt < ALTITUDE_MIN_SAFE_M:
@@ -725,21 +750,42 @@ def seek_altitude(self):
if abs(error) <= ALTITUDE_TOLERANCE_M:
self._log("info",
f"altitude {alt:.2f}m, target {TARGET_ALTITUDE_M:.2f}m "
                    f"(within {ALTITUDE_TOLERANCE_M:.2f}m). Holding here.")
                self.publish(0.0, 0.0, z=Z_NEUTRAL)
                    f"(within {ALTITUDE_TOLERANCE_M:.2f}m). Switching back to ALT_HOLD.")
                self.publish(0.0, self.yaw_to(self.gate_heading), z=Z_NEUTRAL)
                if not self.set_mode("ALT_HOLD"):
                    # STABILIZE does not hold depth -- continuing the rest of
                    # the run believing z=neutral means "depth held" would be
                    # false. Treat this as seriously as any other mode failure.
                    raise Abort("could not switch back to ALT_HOLD after SEEK_ALTITUDE")
return

z = clamp(Z_NEUTRAL + ALTITUDE_SIGN * ALTITUDE_KP * error,
Z_NEUTRAL - ALTITUDE_Z_MAX, Z_NEUTRAL + ALTITUDE_Z_MAX)

            now = time.time()
            if stall_ref_alt is None:
                stall_ref_alt, stall_ref_time = alt, now
            moved = abs(alt - stall_ref_alt)
            elapsed = now - stall_ref_time
            rate = f"{(alt - stall_ref_alt) / elapsed * 1000:+.0f}mm/s" if elapsed > 0.5 else "measuring..."
            if elapsed > ALTITUDE_STALL_CHECK_S:
                if moved < ALTITUDE_STALL_MIN_MOVE_M:
                    self._log("warn",
                        f"commanding z={z:.0f} (delta {z - Z_NEUTRAL:+.0f}) for "
                        f"{elapsed:.0f}s but altitude only moved {moved * 1000:.0f}mm "
                        f"-- not enough thrust? ALTITUDE_SIGN backwards? Consider "
                        f"raising ALTITUDE_KP/ALTITUDE_Z_MAX.")
                stall_ref_alt, stall_ref_time = alt, now   # start a fresh measurement window

# Deliberately NOT labelled ascend/descend: which way z actually
# moves the vehicle is exactly the unverified thing being
# watched here. Read the trend of `altimeter` itself against
# the sign of `z - neutral` to find out, don't trust a label.
self.log_every("seek_altitude", 1.0, lambda: (
f"  altimeter {alt:.2f}m  target {TARGET_ALTITUDE_M:.2f}m  "
                f"error {error:+.2f}m  age {age:.2f}s  "
                f"error {error:+.2f}m  rate {rate}  age {age:.2f}s  "
f"z {z:.0f} (neutral {Z_NEUTRAL:.0f}, delta {z - Z_NEUTRAL:+.0f})"))
            self.tick(0.0, 0.0, z=z)
            self.tick(0.0, self.yaw_to(self.gate_heading), z=z)

def settle(self):
"""Hold heading, no forward thrust. There is no depth logic here:
