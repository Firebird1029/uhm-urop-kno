#!/usr/bin/env python

from __future__ import print_function

import rospy
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointClear, WaypointPush, WaypointSetCurrent
from kno.srv import RauvSimpleCmd, RauvMode

# Create the waypoints plan in QGroundControl, export to a .plan file, then copy and paste the "items: [...]" here,
# replacing "items: [...]" with "items = [...]", "true" with "True", and "None" with "None".
# Make sure there is no trailing comma at the end, i.e. replace "items = [...]," with "items = [...]"
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
            -35.36301074,
            149.16486098,
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
        "doJumpId": 3,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36286853,
            149.1652304,
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
        "doJumpId": 4,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36300568,
            149.16564016,
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
        "doJumpId": 5,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36319333,
            149.16577103,
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
        "doJumpId": 6,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36338795,
            149.16569508,
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
        "doJumpId": 7,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36353457,
            149.16535709,
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
        "doJumpId": 8,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.3635001,
            149.16508081,
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
        "doJumpId": 9,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36336739,
            149.1648063,
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
        "doJumpId": 10,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36313012,
            149.16447876,
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
        "doJumpId": 11,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36294925,
            149.16423504,
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
        "doJumpId": 12,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36296813,
            149.16396975,
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
        "doJumpId": 13,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36308899,
            149.16375491,
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
        "doJumpId": 14,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36329471,
            149.16361607,
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
        "doJumpId": 15,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36340201,
            149.16361346,
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
        "doJumpId": 16,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.3635185,
            149.1637027,
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
        "doJumpId": 17,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36350612,
            149.16395192,
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
        "doJumpId": 18,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36343732,
            149.16411508,
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
        "doJumpId": 19,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36332884,
            149.16428517,
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
        "doJumpId": 20,
        "frame": 3,
        "params": [
            0,
            0,
            0,
            None,
            -35.36330458,
            149.16444946,
            -50
        ],
        "type": "SimpleItem"
    },
    {
        "autoContinue": True,
        "command": 20,
        "doJumpId": 21,
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


def figureEight():
    rospy.init_node("rauv_ex_5_figure_eight_waypoints", anonymous=True)

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
    figureEight()
