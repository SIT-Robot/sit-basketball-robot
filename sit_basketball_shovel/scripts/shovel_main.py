#!/usr/bin/env python3

from shovel_controller import ShovelController
from shovel_node import ShovelNode
import rospy
import sys


if __name__ == '__main__':
    rospy.init_node('basketball_shovel',sys.argv)
    shovel_controller = ShovelController()
    shovel_node = ShovelNode(shovel_controller)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Exit...')