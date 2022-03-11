import rospy
from sensor_msgs.msg import Imu as ImuMsg
from geometry_msgs.msg import Quaternion, Vector3
from typing import *
from tf.transformations import euler_from_quaternion


class Imu:
    """
    IMU类订阅/imu_raw话题消息
    """
    def __init__(self):
        self.__imu_subscriber = rospy.Subscriber('/imu_raw', ImuMsg, self.__imu_cb)
        self.__orientation: Quaternion = Quaternion()
        self.__angular_velocity: Vector3 = Vector3()
        self.__linear_acceleration: Vector3 = Vector3()

    def __imu_cb(self, imu_msg: ImuMsg):
        # 不断更新IMU数据
        self.__orientation = imu_msg.orientation
        self.__angular_velocity = imu_msg.angular_velocity
        self.__linear_acceleration = imu_msg.linear_acceleration

    """
    IMU信息的获取
    """

    @property
    def orientation(self) -> Tuple[float, float, float, float]:
        """
        获取四元数姿态
        """
        x = self.__orientation.x
        y = self.__orientation.y
        z = self.__orientation.z
        w = self.__orientation.w
        return x, y, z, w

    @property
    def euler(self) -> Tuple[float, float, float]:
        """
        获取欧拉角姿态
        翻滚角,俯仰角，偏航角
        """
        roll, pitch, yaw = euler_from_quaternion(self.orientation)
        return roll, pitch, yaw

    @property
    def angular_velocity(self) -> Tuple[float, float, float]:
        """
        获取角速度
        """
        x = self.__angular_velocity.x
        y = self.__angular_velocity.y
        z = self.__angular_velocity.z
        return x, y, z

    @property
    def linear_acceleration(self) -> Tuple[float, float, float]:
        """
        获取线加速度
        """
        x = self.__linear_acceleration.x
        y = self.__linear_acceleration.y
        z = self.__linear_acceleration.z
        return x, y, z
