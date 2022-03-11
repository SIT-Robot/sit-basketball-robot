#!/usr/bin/env python3

import sys

import rospy

from shoot_controller import ShootController
from shoot_node import ShootNode

if __name__ == '__main__':
    rospy.init_node('basketball_shoot', sys.argv)

    sc = ShootController()
    ShootNode(sc)

    try:
        rospy.spin()
    except KeyboardInterrupt as e:
        rospy.loginfo(e)
