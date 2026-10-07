import L1_log as log
import L1_lidar as lidar
import L2_vector as vec
from time import sleep

sensor = lidar.Lidar()
sensor.connect()
sensor.run()
sleep(2)  # let the first scan arrive

while True:
    scan = sensor.get()                                  # get a scan from the sensor
    closest = vec.getNearest(scan)                       # find the nearest obstacle in that scan: [r, alpha]
    xy = vec.polar2cart(closest[0], closest[1])          # convert to cartesian: [x, y]
    log.tmpFile(closest[0], "nearest_distance.txt")          # write the distance (m) to its own file
    log.tmpFile(closest[1], "nearest_angle.txt")             # write the angle (deg) to its own file
    log.tmpFile(xy[0], "nearest_x.txt")                      # write x (m) to its own file
    log.tmpFile(xy[1], "nearest_y.txt")                      # write y (m) to its own file
    sleep(0.1)                                           # sleep