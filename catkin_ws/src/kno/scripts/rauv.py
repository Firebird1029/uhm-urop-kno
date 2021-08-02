#!/usr/bin/env python

from __future__ import print_function

import rospy
from mavros_msgs.msg import OverrideRCIn, ParamValue
from mavros_msgs.srv import CommandBool, WaypointClear, SetMode, ParamGet, ParamSet
from kno.msg import RauvMotion
from kno.srv import RauvSimpleCmd, RauvSimpleCmdResponse, RauvMode, RauvModeResponse


# ROS Services


def handleInitCall(req):
    rospy.logdebug("Initiating RAUV in RAUV node.")

    try:
        # set the SYSID_MYGCS MAVLink parameter to 1 to allow ROS to control the RAUV.
        rospy.ServiceProxy("/mavros/param/get", ParamGet)("SYSID_MYGCS")
        rospy.ServiceProxy("/mavros/param/set", ParamSet)("SYSID_MYGCS", ParamValue(1, 0.0))

        mpgRes = rospy.ServiceProxy("/mavros/param/get", ParamGet)("SYSID_MYGCS")
        rospy.loginfo("SYSID_MYGCS set to: %s" % mpgRes.value.integer)

        # set mode to Manual
        msmRes = rospy.ServiceProxy("/mavros/set_mode", SetMode)(custom_mode="MANUAL")

        # clear waypoints
        mmcRes = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)()

        if (mpgRes.value.integer != 1 or not msmRes.mode_sent or not mmcRes.success):
            rospy.logerr("RAUV failed to initiate.")
            return RauvSimpleCmdResponse(False)

        rospy.loginfo("RAUV initialization successful.")
        return RauvSimpleCmdResponse(True)
    except (rospy.ServiceException, rospy.ROSInterruptException) as e:
        rospy.logerr("RAUV failed to initiate.")
        return RauvSimpleCmdResponse(False)


def handleArmCall(req):
    rospy.logdebug("Arming RAUV.")
    rospy.wait_for_service("/mavros/cmd/arming")
    try:
        res = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)(True)
        rospy.loginfo("RAUV armed successfully.")
        return RauvSimpleCmdResponse(True)
    except (rospy.ServiceException, rospy.ROSInterruptException) as e:
        rospy.logerr("Service call failed: %s" % e)
        return RauvSimpleCmdResponse(False)


def handleDisarmCall(req):
    rospy.logdebug("Disarming RAUV.")
    rospy.wait_for_service("/mavros/cmd/arming")
    try:
        res = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)(False)
        rospy.loginfo("RAUV disarmed successfully.")
        return RauvSimpleCmdResponse(True)
    except (rospy.ServiceException, rospy.ROSInterruptException) as e:
        rospy.logerr("Service call failed: %s" % e)
        return RauvSimpleCmdResponse(False)


def handleModeCall(req):
    rospy.logdebug("Changing RAUV flight mode to %s." % req.mode)
    rospy.wait_for_service("/mavros/set_mode")
    try:
        res = rospy.ServiceProxy("/mavros/set_mode", SetMode)(custom_mode=req.mode)
        if not res.mode_sent:
            rospy.logwarn("RAUV flight mode change error! Failed to set flight mode to %s." % req.mode)
            return RauvModeResponse(False)
        rospy.loginfo("RAUV flight mode successfully changed to %s." % req.mode)
        return RauvModeResponse(True)
    except (rospy.ServiceException, rospy.ROSInterruptException) as e:
        rospy.logerr("Service call failed: %s" % e)
        return RauvModeResponse(False)


# ROS Subscriptions


def handleMotionMsg(msg):
    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    # -1 for no change, 0 for release --> 65535 for no change, 0 for release
    channels = [msg.pitch, msg.roll, msg.throttle,
                msg.yaw, msg.forward, msg.lateral,
                msg.pan, msg.tilt]
    for i, c in enumerate(channels):
        if c < 0:
            channels[i] = 65535
    rospy.logdebug(channels)

    pub.publish(channels=channels)


# Main ROS node code


def rauv_server():
    rospy.init_node("rauv")
    initService = rospy.Service("rauv/init", RauvSimpleCmd, handleInitCall)
    armService = rospy.Service("rauv/arm", RauvSimpleCmd, handleArmCall)
    disarmService = rospy.Service("rauv/disarm", RauvSimpleCmd, handleDisarmCall)
    modeService = rospy.Service("rauv/mode", RauvMode, handleModeCall)

    motionSubscription = rospy.Subscriber("rauv/motion", RauvMotion, handleMotionMsg)
    rospy.loginfo("RAUV node ready.")
    rospy.spin()


if __name__ == "__main__":
    try:
        rauv_server()
    except rospy.ROSInterruptException:
        pass
