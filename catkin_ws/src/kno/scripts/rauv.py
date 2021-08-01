#!/usr/bin/env python

from __future__ import print_function

import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, WaypointClear
from kno.msg import RauvMotion
from kno.srv import RauvSimpleCmd, RauvSimpleCmdResponse
from set_sysid_param import setSysIdParam


def handleInitCall(req):
    rospy.loginfo("Initiating RAUV in RAUV node.")

    try:
        # set the SYSID_MYGCS MAVLink parameter to 1 to allow ROS to control the RAUV.
        setSysIdParam(1)

        # clear waypoints
        rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)()

        return RauvSimpleCmdResponse(True)
    except:
        rospy.logerr("RAUV failed to initiate.")
        return RauvSimpleCmdResponse(False)


def handleArmCall(req):
    rospy.loginfo("Arming RAUV.")
    rospy.wait_for_service("/mavros/cmd/arming")
    try:
        res = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)(True)
        return RauvSimpleCmdResponse(True)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return RauvSimpleCmdResponse(False)


def handleDisarmCall(req):
    rospy.loginfo("Disarming RAUV.")
    rospy.wait_for_service("/mavros/cmd/arming")
    try:
        res = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)(False)
        return RauvSimpleCmdResponse(True)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return RauvSimpleCmdResponse(False)


def handleMotionMsg(msg):
    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    # -1 for no change, 0 for release --> 65535 for no change, 0 for release
    channels = [msg.pitch, msg.roll, msg.throttle,
                msg.yaw, msg.forward, msg.lateral,
                msg.pan, msg.tilt]
    for i, c in enumerate(channels):
        if c < 0:
            channels[i] = 65535
    print(channels)

    pub.publish(channels=channels)


def rauv_server():
    rospy.init_node("rauv")
    initService = rospy.Service("rauv/init", RauvSimpleCmd, handleInitCall)
    armService = rospy.Service("rauv/arm", RauvSimpleCmd, handleArmCall)
    disarmService = rospy.Service("rauv/disarm", RauvSimpleCmd, handleDisarmCall)

    motionSubscription = rospy.Subscriber("rauv/motion", RauvMotion, handleMotionMsg)
    rospy.loginfo("RAUV node ready.")
    rospy.spin()


if __name__ == "__main__":
    try:
        rauv_server()
    except rospy.ROSInterruptException:
        pass
