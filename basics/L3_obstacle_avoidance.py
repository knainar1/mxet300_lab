#!/usr/bin/python3
"""
obstacle_avoidance.py  -  simple SICK-lidar obstacle avoidance for the SCUTTLE.

  Path clear ahead     -> drive forward
  Obstacle ahead       -> spin toward the side with more open space
  Obstacle very close  -> back up briefly, then turn
  Ctrl-C / lidar dies  -> motors stop

Uses:  L1_lidar.Lidar  (SICK TiM561, angles -135..+135 deg, distances in m)
       L1_motor.sendLeft / sendRight  (-1..1)
"""

import time
import numpy as np
from L1_lidar import Lidar
import L1_motor as motor

# ---------------------------------------------------------------- parameters
STOP_DIST   = 1.0   # m   front obstacle closer than this -> turn
CLEAR_DIST  = 0.65   # m   once turning, keep turning until front is beyond this
DANGER_DIST = 0.25   # m   closer than this -> back up first
MAX_DIST    = 3.0    # m   ignore anything farther than this
MIN_VALID   = 0.10   # m   readings below this are sensor junk (matches L1_lidar.min_distance)

FRONT_HALF  = 30     # deg front zone is +/- this around forward
SIDE_MAX    = 90     # deg side zones run from FRONT_HALF out to this

DRIVE_SPEED = 0.65   # motor command, forward
TURN_SPEED  = 0.45   # motor command, spin in place
BACK_SPEED  = 0.40   # motor command, reverse
BACK_TIME   = 0.5    # s
LOOP_DELAY  = 0.05   # s

# --- mounting / wiring checks (test these with the wheels off the ground!) ---
LEFT_IS_POSITIVE   = True   # True if +angle is the robot's LEFT. Flip if left/right are swapped.
FORWARD_OFFSET_DEG = 0.0    # lidar angle that points straight ahead (0 if mounted forward)
LEFT_DIR  = 1               # set to -1 if the left wheel spins the wrong way
RIGHT_DIR = 1               # set to -1 if the right wheel spins the wrong way


# ---------------------------------------------------------------- helpers
def get_scan(lidar):
    """Return (distances_m, angles_deg) with 0 deg = forward, or None if no data yet.

    Reads lidar.ds / lidar.angles directly (rather than lidar.get()) so that
    zero/dropout readings can't masquerade as the closest obstacle.
    """
    ds = lidar.ds
    if ds is None:
        return None

    dist = np.array(ds, dtype=float)
    ang = np.array(lidar.angles, dtype=float)

    if len(dist) != len(ang):               # unexpected packet size -> fall back to get()
        data = lidar.get()
        if data is None:
            return None
        dist, ang = data[:, 0], data[:, 1]

    ang = ang - FORWARD_OFFSET_DEG
    ang = (ang + 180.0) % 360.0 - 180.0     # wrap to (-180, 180]
    if not LEFT_IS_POSITIVE:
        ang = -ang
    return dist, ang


def min_in_range(dist, ang, lo, hi):
    """Closest valid reading with lo <= angle <= hi. MAX_DIST if nothing valid."""
    mask = (ang >= lo) & (ang <= hi) & (dist > MIN_VALID) & (dist < MAX_DIST)
    return float(dist[mask].min()) if np.any(mask) else MAX_DIST


def clearances(dist, ang):
    front = min_in_range(dist, ang, -FRONT_HALF, FRONT_HALF)
    left  = min_in_range(dist, ang,  FRONT_HALF,  SIDE_MAX)
    right = min_in_range(dist, ang, -SIDE_MAX,   -FRONT_HALF)
    return front, left, right


def drive(left, right):
    motor.sendLeft(LEFT_DIR * left)
    motor.sendRight(RIGHT_DIR * right)


def stop():
    drive(0, 0)


# ---------------------------------------------------------------- main loop
def main():
    lidar = Lidar()
    lidar.connect()
    lidar_thread = lidar.run()
    time.sleep(1)                      # let the first scans arrive

    turning = None                     # None, "left" or "right" (stops left/right dithering)
    print("Obstacle avoidance running. Ctrl-C to stop.")

    try:
        while True:
            if not lidar_thread.is_alive():
                print("Lidar thread died - stopping.")
                break

            scan = get_scan(lidar)
            if scan is None:
                stop()
                time.sleep(LOOP_DELAY)
                continue

            front, left, right = clearances(*scan)

            if front < DANGER_DIST:
                print(f"DANGER  front={front:.2f} -> back up")
                drive(-BACK_SPEED, -BACK_SPEED)
                time.sleep(BACK_TIME)
                stop()
                turning = None

            elif front < STOP_DIST or (turning and front < CLEAR_DIST):
                if turning is None:
                    turning = "left" if left > right else "right"
                print(f"BLOCKED front={front:.2f} L={left:.2f} R={right:.2f} -> turn {turning}")
                if turning == "left":
                    drive(-TURN_SPEED, TURN_SPEED)
                else:
                    drive(TURN_SPEED, -TURN_SPEED)

            else:
                turning = None
                print(f"CLEAR   front={front:.2f} -> forward")
                drive(DRIVE_SPEED, DRIVE_SPEED)

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        stop()
        lidar.kill(lidar_thread)


if __name__ == "__main__":
    main()