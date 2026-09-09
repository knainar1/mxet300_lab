import L1_log as log
import L1_ina as ina
import time

while True:
    voltage = ina.readVolts()
    log.tmpFile(voltage, "volttemp.txt")
    time.sleep(0.5)
