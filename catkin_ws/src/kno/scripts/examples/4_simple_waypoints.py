#!/usr/bin/env python

from __future__ import print_function

import rospy
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointClear, WaypointPush, WaypointSetCurrent
from kno.srv import RauvSimpleCmd, RauvMode

# create the waypoints plan in QGroundControl, export to a .plan file, then copy and paste the items: [...] here, replacing "items: [...]" with "items = [...]"
items = [
    {
        "autoContinue": True,
        "command": 22,
        "doJumpId": 1,
        "frame": 3,
        "params": [
            15,
            0,
            0,
            None,
            0,
            0,
            -50
        ],
        "type": "SimpleItem"
    },
    {
        "AMSLAltAboveTerrain": None,
        "Altitude": -50,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": 16,
        "doJumpId": 2,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36264004,
            149.164939,
            -50
        ],
        "type": "SimpleItem"
    },
    {
        "autoContinue": True,
        "command": 20,
        "doJumpId": 3,
        "frame": 2,
        "params": [
            0,
            0,
            0,
            0,
            0,
            0,
            0
        ],
        "type": "SimpleItem"
    }
]


# convert .plan waypoints to mavros waypoints
def convertQGCPlanToWaypoints(planItems):
    def convertQGCPlanItemtoWaypoint(item):
        if item["type"] != "SimpleItem":
            rospy.logwarn("QGroundControl waypoints that are not simple are not yet supported.")

        # https://mavlink.io/en/messages/common.html#MISSION_ITEM
        # ['frame', 'command', 'is_current', 'autocontinue', 'param1', 'param2', 'param3', 'param4', 'x_lat', 'y_long', 'z_alt']
        return Waypoint(*[item["frame"], item["command"], 0, item["autoContinue"],
                          item["params"][0], item["params"][1], item["params"][2], item["params"][3],
                          item["params"][4], item["params"][5], item["params"][6]])

    return map(convertQGCPlanItemtoWaypoint, planItems)


def simpleWaypoints():
    rospy.init_node("rauv_ex_4_simple_waypoints", anonymous=True)

    rospy.ServiceProxy("/rauv/init", RauvSimpleCmd)()

    rospy.ServiceProxy("/rauv/arm", RauvSimpleCmd)()

    waypoints = convertQGCPlanToWaypoints(items)
    print(waypoints)

    try:
        rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)()
        rospy.ServiceProxy("/mavros/mission/push", WaypointPush)(waypoints=waypoints)
        rospy.ServiceProxy("/mavros/mission/set_current", WaypointSetCurrent)(wp_seq=0)

        rospy.ServiceProxy("/rauv/mode", RauvMode)("AUTO")
    except rospy.ServiceException as e:
        rospy.logerror(e)


if __name__ == "__main__":
    simpleWaypoints()
