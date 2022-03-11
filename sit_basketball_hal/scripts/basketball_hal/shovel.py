import rospy
from sit_basketball_shovel_srv.srv import ShovelControl
import enum


class ControlType(enum.IntEnum):
    """
    控制类型的枚举类
    """
    STOP = 0x00
    UP = 0x01
    DOWN = 0x02


class Shovel:
    """
    起落架的控制
    """

    def __init__(self):
        self.shovel_server_proxy = rospy.ServiceProxy('/basketball_shovel/ShovelControl', ShovelControl)
        rospy.loginfo('等待连接起落架服务器')
        self.shovel_server_proxy.wait_for_service()
        rospy.loginfo('起落架服务器连接成功')

    def up(self):
        """
        起落架上升
        """
        self.shovel_server_proxy.call(ControlType.UP)

    def down(self):
        """
        起落架下降
        """
        self.shovel_server_proxy.call(ControlType.DOWN)

    def stop(self):
        """
        起落架停止
        """
        self.shovel_server_proxy.call(ControlType.STOP)
