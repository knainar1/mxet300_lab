# L3_log_speeds.py
# Reads wheel speeds [PDL, PDR] and chassis speeds [xdot, thetadot] from the
# encoders, prints them to the terminal, and logs each one to its own file
# for Node-RED to read.
#
# Run this BEFORE L3_path_template.py, in its own terminal.

import time                                 # library for time access
import L2_kinematics as kin                 # local library for forward kinematics
import L1_log as log                        # local library for logging to files

while True:
    # One call takes one encoder measurement. It returns the chassis speeds
    # and also stores the wheel speeds in kin.pdCurrents, so both come from
    # the same moment in time.
    motion = kin.getMotion()                # [xdot (m/s), thetadot (rad/s)]
    xdot = motion[0]
    thetadot = motion[1]

    pdl = kin.pdCurrents[0]                 # left wheel speed (rad/s)
    pdr = kin.pdCurrents[1]                 # right wheel speed (rad/s)

    # Print all four to the terminal
    print("xdot(m/s): {:.3f}\tthetadot(rad/s): {:.3f}\tpdl(rad/s): {:.3f}\tpdr(rad/s): {:.3f}".format(
        xdot, thetadot, pdl, pdr))

    # Write each value to its own file (use these exact names in Node-RED)
    log.tmpFile(xdot, "xdot.txt")
    log.tmpFile(thetadot, "thetadot.txt")
    log.tmpFile(pdl, "pdl.txt")
    log.tmpFile(pdr, "pdr.txt")

    time.sleep(0.2)                         # give Node-RED time to read; lets Ctrl+C register