#!/usr/bin/python3
"""
L3_lidar_mapping.py  -  build a 2D occupancy-grid map of the room with the SICK lidar.

  Encoders           -> wheel travel -> robot pose  [x, y, theta]   (dead reckoning)
  Lidar scan         -> polar2cart -> rotate by theta -> add robot x,y -> global points
  Global points      -> occupancy grid (cells along each beam = free, end of beam = wall)
  Map                -> saved to /tmp as a .pgm image + .csv of wall points
                     -> wall points streamed to Node-RED bubble chart (same as lidar_driving.py)

  DRIVE_MODE = "manual" : drive the robot yourself (joystick flow, or L3_noderedControl.py)
                          while this program only listens and builds the map.
  DRIVE_MODE = "auto"   : the robot wanders on its own using the same logic as
                          L3_obstacle_avoidance.py while it maps.

  Ctrl-C -> motors stop, map is saved.

Uses:  L1_lidar.Lidar           (SICK TiM561, angles -135..+135 deg, distances in m)
       L1_encoder.readShaftPositions
       L2_kinematics            (R, L, phiTravels)
       L2_vector                (polar2cart, rotate, sumVec)
       L1_log                   (tmpFile / stringTmpFile for Node-RED)
       L3_obstacle_avoidance    (only when DRIVE_MODE = "auto")

View the saved map:  copy /tmp/lidar_map.pgm to your laptop and open it (GIMP, IrfanView,
VS Code image preview, or python PIL). White = free, black = wall, grey = unknown,
red-ish dot is not drawn - robot path is written to /tmp/lidar_path.csv.
"""

import time
import socket
import numpy as np
from threading import Thread, Lock

from L1_lidar import Lidar
import L1_encoder as enc
import L2_kinematics as kin
import L2_vector as vec
import L1_log as log

# ---------------------------------------------------------------- parameters
DRIVE_MODE   = "auto"   # "manual" or "auto"

MAP_SIZE     = 12.0       # m   width/height of the square map (robot starts in the middle)
CELL_SIZE    = 0.05       # m   size of one grid cell (5 cm)
MAX_RANGE    = 3.0        # m   ignore lidar returns farther than this (matches L1_lidar.max_distance)
MIN_VALID    = 0.10       # m   readings below this are sensor junk (matches L1_lidar.min_distance)
BEAM_STEP    = 3          # use every Nth of the 811 lidar beams (3 -> ~270 beams per scan)

# log-odds occupancy update values
HIT          = 0.85       # added to a cell when a beam ends in it
MISS         = -0.40      # added to a cell when a beam passes through it
LOGODDS_MAX  = 5.0        # clamp so the map can still change its mind
OCCUPIED     = 1.5        # log-odds above this counts as a wall when exporting

MAP_PERIOD   = 0.25       # s   how often a scan is added to the map
MAX_TURN_FOR_SCAN = 0.6   # rad/s  skip scans while spinning fast (pose is less reliable)
PUBLISH_PERIOD = 1.0      # s   how often the map is sent to Node-RED / saved
SAVE_PATH    = "/tmp/lidar_map"

# Node-RED bubble chart (same ports as lidar_driving.py / flow_lidar_driving.json)
NODERED_IP   = "127.0.0.1"
NODERED_PORT = 3555
MAX_CHART_POINTS = 1500   # UDP packets can only be so big

# --- mounting checks (same meaning as in L3_obstacle_avoidance.py) ---
LEFT_IS_POSITIVE   = True  # True if +lidar angle is the robot's LEFT
FORWARD_OFFSET_DEG = 0.0   # lidar angle that points straight ahead
LIDAR_X_OFFSET     = 0.0   # m   how far the lidar sits in front of the wheel axle


# ---------------------------------------------------------------- odometry
class Odometry:
    """Integrates encoder travel into a global pose [x (m), y (m), theta (rad)].

    Runs in its own thread so the pose keeps updating while the lidar
    math is being done. Start pose is (0, 0, 0) facing +x.
    """

    def __init__(self, period=0.02):
        self.period = period
        self.pose = np.array([0.0, 0.0, 0.0])
        self.thetaDot = 0.0
        self.lock = Lock()
        self.stop = False

    def get(self):
        with self.lock:
            return self.pose.copy()

    def _loop(self):
        encoders_t1 = enc.readShaftPositions()              # shaft angles in degrees
        t1 = time.monotonic()
        while not self.stop:
            time.sleep(self.period)
            encoders_t2 = enc.readShaftPositions()
            t2 = time.monotonic()

            wheelTravel = kin.phiTravels(encoders_t1, encoders_t2)   # [deg, deg] wheel rotation
            arc = np.radians(wheelTravel) * kin.R                    # [m, m] distance each wheel rolled
            dS = (arc[0] + arc[1]) / 2                               # forward travel of the center (m)
            dTheta = (arc[1] - arc[0]) / (2 * kin.L)                 # change in heading (rad)

            with self.lock:
                theta = self.pose[2]
                midTheta = theta + dTheta / 2                        # use the heading halfway through the step
                self.pose[0] += dS * np.cos(midTheta)
                self.pose[1] += dS * np.sin(midTheta)
                self.pose[2] = (theta + dTheta + np.pi) % (2 * np.pi) - np.pi   # wrap to (-pi, pi]
                dt = t2 - t1
                self.thetaDot = dTheta / dt if dt > 0 else 0.0

            encoders_t1 = encoders_t2
            t1 = t2

    def run(self):
        thread = Thread(target=self._loop, daemon=True)
        thread.start()
        return thread


# ---------------------------------------------------------------- occupancy grid
class OccupancyGrid:
    """Square grid of log-odds values. 0 = unknown, + = wall, - = free."""

    def __init__(self, size=MAP_SIZE, cell=CELL_SIZE):
        self.cell = cell
        self.n = int(round(size / cell))
        self.origin = self.n // 2                       # grid index of world (0, 0)
        self.grid = np.zeros((self.n, self.n))          # grid[row, col] = grid[y, x]

    def world2grid(self, xy):
        """[N,2] world points (m) -> [N,2] integer cell indices (col, row)."""
        return np.floor(np.asarray(xy) / self.cell).astype(int) + self.origin

    def grid2world(self, ij):
        """[N,2] cell indices (col, row) -> [N,2] world coordinates of the cell centers (m)."""
        return (np.asarray(ij) - self.origin + 0.5) * self.cell

    def inBounds(self, ij):
        return (ij[:, 0] >= 0) & (ij[:, 0] < self.n) & (ij[:, 1] >= 0) & (ij[:, 1] < self.n)

    def addScan(self, robotXY, hitsXY, freeXY):
        """robotXY: [x,y] of the sensor.  hitsXY: [N,2] beam end points that hit something.
        freeXY: [M,2] beam end points of max-range beams (no hit, but the space is empty)."""
        start = self.world2grid(robotXY)
        ends = [(p, True) for p in self.world2grid(hitsXY)] + [(p, False) for p in self.world2grid(freeXY)]

        freeCells = []
        for end, isHit in ends:
            line = bresenham(start, end)
            freeCells.append(line[:-1])                 # every cell before the end point is free
            if not isHit:
                freeCells.append(line[-1:])

        if freeCells:
            free = np.unique(np.vstack(freeCells), axis=0)   # count each free cell once per scan
            free = free[self.inBounds(free)]
            self.grid[free[:, 1], free[:, 0]] += MISS

        if len(hitsXY):
            hits = np.unique(self.world2grid(hitsXY), axis=0)
            hits = hits[self.inBounds(hits)]
            self.grid[hits[:, 1], hits[:, 0]] += HIT

        np.clip(self.grid, -LOGODDS_MAX, LOGODDS_MAX, out=self.grid)

    def occupiedPoints(self):
        """World coordinates [N,2] of every cell considered a wall."""
        rows, cols = np.where(self.grid > OCCUPIED)
        return self.grid2world(np.column_stack((cols, rows)))

    def savePGM(self, fileName, path=None):
        """Save the map as a greyscale image. White = free, black = wall, grey = unknown.
        PGM needs no extra libraries and opens in most image viewers."""
        prob = 1 - 1 / (1 + np.exp(self.grid))          # log-odds -> probability of wall
        img = (255 * (1 - prob)).astype(np.uint8)
        img = np.flipud(img)                            # so +y is up in the image

        if path is not None and len(path):
            ij = self.world2grid(path)
            ij = ij[self.inBounds(ij)]
            img[self.n - 1 - ij[:, 1], ij[:, 0]] = 128  # draw the robot path in mid grey

        with open(fileName, 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (self.n, self.n))
            f.write(img.tobytes())


def bresenham(start, end):
    """All grid cells on the line from start to end (both [col, row]), inclusive."""
    x0, y0 = int(start[0]), int(start[1])
    x1, y1 = int(end[0]), int(end[1])
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return np.array(cells)


# ---------------------------------------------------------------- lidar helpers
def get_scan(lidar):
    """Return (distances_m, angles_deg) with 0 deg = forward and +angle = left, or None."""
    ds = lidar.ds
    if ds is None:
        return None

    dist = np.array(ds, dtype=float)
    ang = np.array(lidar.angles, dtype=float)
    if len(dist) != len(ang):                   # unexpected packet size -> skip this scan
        return None

    dist, ang = dist[::BEAM_STEP], ang[::BEAM_STEP]
    ang = ang - FORWARD_OFFSET_DEG
    if not LEFT_IS_POSITIVE:
        ang = -ang
    return dist, ang


def scan2global(dist, ang, pose):
    """Turn one scan into global points using the L2_vector functions.

    Returns (sensorXY, hitsXY, freeXY):
      hitsXY - beams that hit something within range
      freeXY - beams that saw nothing, clipped to MAX_RANGE (used only to clear space)
    """
    x, y, theta = pose
    sensorXY = vec.sumVec(vec.rotate(np.array([LIDAR_X_OFFSET, 0.0]), theta), np.array([x, y]))

    hits, free = [], []
    for d, a in zip(dist, ang):
        if d < MIN_VALID:                       # 0 / tiny readings = no return
            continue
        isHit = d < MAX_RANGE
        local = vec.polar2cart(min(d, MAX_RANGE), a)             # robot frame [x, y]
        local = local + np.array([LIDAR_X_OFFSET, 0.0])          # move to the axle center
        world = vec.sumVec(vec.rotate(local, theta), np.array([x, y]))  # global frame
        (hits if isHit else free).append(world)

    hitsXY = np.array(hits).reshape(-1, 2)
    freeXY = np.array(free).reshape(-1, 2)
    return sensorXY, hitsXY, freeXY


# ---------------------------------------------------------------- Node-RED output
def format_row(point, r=2):
    """Same bubble-chart row format as lidar_driving.py."""
    x, y = point
    return '{x: ' + str(round(x, 2)) + ', y: ' + str(round(y, 2)) + ', r:' + str(r) + '},'


def publish(sock, grid, pose):
    walls = grid.occupiedPoints()
    if len(walls) > MAX_CHART_POINTS:           # thin the points out so the packet fits
        walls = walls[::int(np.ceil(len(walls) / MAX_CHART_POINTS))]

    rows = ''.join(format_row(p) for p in walls)
    rows += format_row(pose[:2], r=6)           # draw the robot as a big dot
    try:
        sock.sendto(rows[:-1].encode('utf-8'), (NODERED_IP, NODERED_PORT))
    except OSError:
        pass

    log.tmpFile(pose[0], "map_x.txt")           # robot pose for Node-RED text widgets
    log.tmpFile(pose[1], "map_y.txt")
    log.tmpFile(np.degrees(pose[2]), "map_theta.txt")


def save(grid, path):
    path = np.array(path).reshape(-1, 3)        # rows of [x, y, theta]
    grid.savePGM(SAVE_PATH + ".pgm", path[:, :2])
    np.save(SAVE_PATH + ".npy", grid.grid)
    np.savetxt(SAVE_PATH + "_walls.csv", grid.occupiedPoints(), delimiter=",", fmt="%.3f", header="x,y", comments="")
    np.savetxt("/tmp/lidar_path.csv", path, delimiter=",", fmt="%.3f",
               header="x,y,theta", comments="")


# ---------------------------------------------------------------- main loop
def main():
    lidar = Lidar()
    lidar.connect()
    lidar_thread = lidar.run()

    odom = Odometry()
    odom.run()

    grid = OccupancyGrid()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    path = []

    if DRIVE_MODE == "auto":
        import L3_obstacle_avoidance as oa      # only needed (and only touches the motors) in auto mode
        turning = None

    time.sleep(1)                               # let the first scans arrive
    print("Mapping running in", DRIVE_MODE, "mode. Ctrl-C to stop and save.")

    lastPublish = time.monotonic()
    try:
        while True:
            if not lidar_thread.is_alive():
                print("Lidar thread died - stopping.")
                break

            scan = get_scan(lidar)
            if scan is None:
                time.sleep(MAP_PERIOD)
                continue

            pose = odom.get()
            path.append(pose)

            # --- add this scan to the map (skip it if spinning fast: the scan smears)
            if abs(odom.thetaDot) < MAX_TURN_FOR_SCAN:
                sensorXY, hitsXY, freeXY = scan2global(*scan, pose)
                grid.addScan(sensorXY, hitsXY, freeXY)

            # --- wander on our own (same decisions as L3_obstacle_avoidance.py)
            if DRIVE_MODE == "auto":
                oaScan = oa.get_scan(lidar)
                if oaScan is not None:
                    front, left, right = oa.clearances(*oaScan)
                    if front < oa.DANGER_DIST:
                        oa.drive(-oa.BACK_SPEED, -oa.BACK_SPEED)
                        time.sleep(oa.BACK_TIME)
                        oa.stop()
                        turning = None
                    elif front < oa.STOP_DIST or (turning and front < oa.CLEAR_DIST):
                        if turning is None:
                            turning = "left" if left > right else "right"
                        if turning == "left":
                            oa.drive(-oa.TURN_SPEED, oa.TURN_SPEED)
                        else:
                            oa.drive(oa.TURN_SPEED, -oa.TURN_SPEED)
                    else:
                        turning = None
                        oa.drive(oa.DRIVE_SPEED, oa.DRIVE_SPEED)

            # --- send to Node-RED and save every so often
            if time.monotonic() - lastPublish > PUBLISH_PERIOD:
                publish(sock, grid, pose)
                save(grid, path)
                print("pose x={:.2f} y={:.2f} th={:.1f}deg  walls={}".format(
                    pose[0], pose[1], np.degrees(pose[2]), len(grid.occupiedPoints())))
                lastPublish = time.monotonic()

            time.sleep(MAP_PERIOD)

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        if DRIVE_MODE == "auto":
            oa.stop()
        odom.stop = True
        lidar.kill(lidar_thread)
        if path:
            save(grid, path)
        print("Map saved to", SAVE_PATH + ".pgm", "and", SAVE_PATH + "_walls.csv")


if __name__ == "__main__":
    main()
