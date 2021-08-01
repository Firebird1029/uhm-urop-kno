#!/usr/bin/env python

from __future__ import print_function

import rospy
from pymavlink import mavutil
import time
import sys


def setSysIdParam(sysId):
    """Set the SYSID_MYGCS MAVLink parameter to allow ROS to control the RAUV."""

    # https://www.ardusub.com/developers/pymavlink.html#read-and-write-parameters

    # ! note that if another program (e.g. mavros, QGroundControl) is using 14551, it will kick off that program
    master = mavutil.mavlink_connection("udpin:0.0.0.0:14551")
    master.wait_heartbeat()

    # Request parameter
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b"SYSID_MYGCS",
        -1
    )

    # Print old parameter value
    message = master.recv_match(type="PARAM_VALUE", blocking=True).to_dict()
    print("OLD name: %s\tvalue: %d" %
          (message["param_id"].decode("utf-8"), message["param_value"]))

    time.sleep(1)

    # Set parameter value
    # Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot.
    # Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM.
    # The parameter variable type is described by MAV_PARAM_TYPE in http://mavlink.org/messages/common.
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b"SYSID_MYGCS",
        sysId,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    # Read ACK
    # IMPORTANT: The receiving component should acknowledge the new parameter value by sending a
    # param_value message to all communication partners.
    # This will also ensure that multiple GCS all have an up-to-date list of all parameters.
    # If the sending GCS did not receive a PARAM_VALUE message within its timeout time,
    # it should re-send the PARAM_SET message.
    message = master.recv_match(type="PARAM_VALUE", blocking=True).to_dict()
    print("SET name: %s\tvalue: %d" %
          (message["param_id"].decode("utf-8"), message["param_value"]))

    time.sleep(1)

    # Request parameter value to confirm
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b"SYSID_MYGCS",
        -1
    )

    # Print new value in RAM
    message = master.recv_match(type="PARAM_VALUE", blocking=True).to_dict()
    print("NEW name: %s\tvalue: %d" %
          (message["param_id"].decode("utf-8"), message["param_value"]))


if __name__ == "__main__":
    if len(sys.argv) == 2:
        sysId = int(sys.argv[1])
        print("Setting SYSID_MYGCS to %s." % sysId)
        setSysIdParam(sysId)
        print("Finished.")
    else:
        print("%s [sys_id]" % sys.argv[0])
