import select
import termios
import tty

import rospy
from typing import *
import sys
from basketball_hal.chassis import Chassis
from basketball_hal.imu import Imu
from basketball_hal.odometry import Odometry
from basketball_hal.shoot import Shoot
from basketball_hal.shovel import Shovel


def get_key() -> str:
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    if select.select([sys.stdin], [], [], 0.1)[0]:
        key_ = sys.stdin.read(1)
    else:
        key_ = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key_


class TeleopController:
    def __init__(self):
        self.chassis = Chassis()
        self.imu = Imu()
        self.odom = Odometry()
        self.shoot = Shoot()
        self.shovel = Shovel()

        self.key_map = {
            'w': self.forward,
            'a': self.left,
            'd': self.right,
            's': self.backward,
            'j': self.up,
        }

    def forward(self):
        self.chassis.expected_vx = 0.2

    def backward(self):
        self.chassis.expected_vx = -0.2

    def left(self):
        self.chassis.expected_vw = 1

    def right(self):
        self.chassis.expected_vw = -1

    def up(self):
        self.shovel.up()

    def down(self):
        self.shovel.down()

    def stop(self):
        self.chassis.expected_vx = 0
        self.chassis.expected_vy = 0
        self.chassis.expected_vw = 0

        self.shovel.down()

    def key_control(self, key: str):
        try:
            self.key_map[key]()
        except KeyError:
            self.stop()


if __name__ == '__main__':
    rospy.init_node('app_node', argv=sys.argv)

    teleop = TeleopController()


    def timer(t):
        key = get_key()
        teleop.key_control(key)


    rospy.Timer(rospy.Duration.from_sec(0.1), timer)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Exit...')
