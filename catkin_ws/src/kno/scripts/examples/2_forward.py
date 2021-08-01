#!/usr/bin/env python

from __future__ import print_function

import rospy
from kno.msg import RauvMotion
from kno.srv import RauvSimpleCmd, RauvMode


def forward():
    rospy.init_node("rauv_ex_2_forward", anonymous=True)

    rospy.ServiceProxy("/rauv/init", RauvSimpleCmd)()

    rospy.ServiceProxy("/rauv/mode", RauvMode)("MANUAL")

    rospy.ServiceProxy("/rauv/arm", RauvSimpleCmd)()

    pub = rospy.Publisher("/rauv/motion", RauvMotion, queue_size=10)
    rate = rospy.Rate(1)  # 10hz
    while not rospy.is_shutdown():
        # -1 for no change, 0 for release
        pub.publish(pitch=0, roll=0, throttle=0,
                    yaw=0, forward=1900, lateral=0,
                    pan=0, tilt=0)
        rate.sleep()


if __name__ == "__main__":
    forward()
