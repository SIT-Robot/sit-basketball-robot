from typing import Tuple

import rospy
from nav_msgs.msg import Odometry as OdometryMsg
from tf.transformations import euler_from_quaternion


class Odometry:
    """
    里程计信息的读取
    """

    def __init__(self):
        # 绝对里程计的数值
        self._odom_x = 0
        self._odom_y = 0
        self._odom_yaw = 0

        # 相对里程计的相对点
        self._start_odom_x = 0
        self._start_odom_y = 0
        self._start_odom_yaw = 0

        # 里程计是否需要重置相对点
        self._need_reset = True

        # 订阅者
        self._sub = rospy.Subscriber('/odom', OdometryMsg, callback=self.odom_cb)

    def odom_cb(self, odom: OdometryMsg):
        # 获取绝对里程计信息
        self._odom_x = odom.pose.pose.position.x
        self._odom_y = odom.pose.pose.position.y
        ox = odom.pose.pose.orientation.x
        oy = odom.pose.pose.orientation.y
        oz = odom.pose.pose.orientation.z
        ow = odom.pose.pose.orientation.w
        _, _, yaw = euler_from_quaternion([ox,oy,oz,ow])
        self._odom_yaw = yaw

        if self._need_reset:
            self.reset()
            self._need_reset = False

    @property
    def odom_x(self) -> float:
        return self._odom_x - self._start_odom_x

    @property
    def odom_y(self) -> float:
        return self._odom_y - self._start_odom_y

    @property
    def odom_yaw(self) -> float:
        return self._odom_yaw - self._start_odom_yaw

    def reset(self):
        self._start_odom_x = self._odom_x
        self._start_odom_y = self._odom_y
        self._start_odom_yaw = self._odom_yaw
