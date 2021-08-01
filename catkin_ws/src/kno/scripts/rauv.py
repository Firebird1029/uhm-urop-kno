#!/usr/bin/env python

from __future__ import print_function

from kno.srv import RauvSimpleCmd, RauvSimpleCmdResponse
import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from set_sysid_param import setSysIdParam


def handleInitCall(req):
    rospy.loginfo("Initiating RAUV in RAUV node.")

    try:
        # set the SYSID_MYGCS MAVLink parameter to 1 to allow ROS to control the RAUV.
        setSysIdParam(1)
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


def handleMotionCall():
    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    rate = rospy.Rate(1)  # 10hz
    while not rospy.is_shutdown():
        channels = [1500, 1500, 1500, 1500, 1900, 1500, 1500, 1500]
        rospy.loginfo(channels)
        pub.publish(channels=channels)
        rate.sleep()


def rauv_server():
    rospy.init_node("rauv")
    initService = rospy.Service("rauv/init", RauvSimpleCmd, handleInitCall)
    armService = rospy.Service("rauv/arm", RauvSimpleCmd, handleArmCall)
    disarmService = rospy.Service("rauv/disarm", RauvSimpleCmd, handleDisarmCall)
    rospy.loginfo("RAUV node ready.")
    rospy.spin()


if __name__ == "__main__":
    rauv_server()
