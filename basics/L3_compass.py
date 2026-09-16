import time
import numpy as np
import board
import adafruit_bno055
import L2_compass_heading as compass
import L1_log as log

def compass_get_heading():
    heading = compass.get_heading()
    heading = (heading + 180) % 360 - 180
    return heading


def get_direction(heading_360):
    if heading_360 >= 337.5 or heading_360 < 22.5:
        direction = "North"
    elif 22.5 <= heading_360 < 67.5:
        direction = "North West"
    elif 67.5 <= heading_360 < 112.5:
        direction = "West"
    elif 112.5 <= heading_360 < 157.5:
        direction = "South West"
    elif 157.5 <= heading_360 < 202.5:
        direction = "South"
    elif 202.5 <= heading_360 < 247.5:
        direction = "South East"
    elif 247.5 <= heading_360 < 292.5:
        direction = "East"
    else:  # 292.5 <= heading_360 < 337.5
        direction = "North East"

    return direction

while True:
    heading = compass_get_heading()        # get the raw -180 to 180 heading
    heading_360 = heading % 360            # convert it to 0-360
    direction = get_direction(heading_360) # convert to a cardinal direction

    log.tmpFile(heading_360, "heading.txt")           # log the numeric heading
    log.stringTmpFile(direction, "direction.txt")     # log the cardinal direction
    print(round(heading_360, 2), direction)
    time.sleep(0.1)

