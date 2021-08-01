#!/usr/bin/env python

from __future__ import print_function

import rospy
from kno.srv import RauvSimpleCmd


def arm():
    rospy.init_node("rauv_ex_1_arm", anonymous=True)

    rospy.ServiceProxy("/rauv/init", RauvSimpleCmd)()

    rospy.ServiceProxy("/rauv/arm", RauvSimpleCmd)()


if __name__ == "__main__":
    arm()
