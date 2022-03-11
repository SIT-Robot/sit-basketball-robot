import rospy
from sit_basketball_shoot_srv.srv import *
from std_srvs.srv import *
import enum


class ShootStatusEnum(enum.IntEnum):
    """表示电磁炮状态的枚举类
    """
    # 未就绪的
    UNREADY = 0x00

    # 已就绪的
    READY = 0x01


class Shoot:
    """
    电磁炮的控制
    """

    def __init__(self):
        self.__charge_proxy = rospy.ServiceProxy('/basketball_shoot/Charge', ShootCharge)
        self.__shoot_proxy = rospy.ServiceProxy('/basketball_shoot/Shoot', Empty)
        self.__status_proxy = rospy.ServiceProxy('/basketball_shoot/Status', ShootStatus)

        rospy.loginfo('正在连接服务器')
        self.__charge_proxy.wait_for_service()
        self.__shoot_proxy.wait_for_service()
        self.__status_proxy.wait_for_service()
        rospy.loginfo('服务器连接成功')

    def charge(self, charge_time: float) -> bool:
        """电磁炮充电
        充电时间(单位s秒)
        """
        req = ShootChargeRequest()

        # 单位换算,ROS服务中的充电时间使用ms毫秒为单位
        req.charge_time = int(charge_time * 1000)
        resp: ShootChargeResponse = self.__charge_proxy.call(req)
        return resp.success

    def shoot(self):
        """
        电磁炮发射
        """
        req = EmptyRequest()
        self.__shoot_proxy.call(req)

    @property
    def status(self):
        """
        获取电磁炮状态
        """
        req = ShootStatusRequest()
        resp: ShootStatusResponse = self.__status_proxy.call(req)

        return {
            0x01: ShootStatusEnum.READY,
            0x00: ShootStatusEnum.UNREADY
        }[resp.status]
