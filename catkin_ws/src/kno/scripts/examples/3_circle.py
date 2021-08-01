#!/usr/bin/env python

from __future__ import print_function

import rospy
from kno.msg import RauvMotion
from kno.srv import RauvSimpleCmd, RauvMode


def circle():
    rospy.init_node("rauv_ex_3_circle", anonymous=True)

    rospy.ServiceProxy("/rauv/init", RauvSimpleCmd)()

    rospy.ServiceProxy("/rauv/mode", RauvMode)("MANUAL")

    rospy.ServiceProxy("/rauv/arm", RauvSimpleCmd)()

    pub = rospy.Publisher("/rauv/motion", RauvMotion, queue_size=10)
    rate = rospy.Rate(1)  # 10hz
    i = 0
    while not rospy.is_shutdown():
        # -1 for no change, 0 for release
        yaw = 1550 if i % 10 == 0 else 0
        pub.publish(pitch=-1, roll=-1, throttle=-1,
                    yaw=yaw, forward=1900, lateral=-1,
                    pan=-1, tilt=-1)
        i += 1
        rate.sleep()


if __name__ == "__main__":
    circle()
