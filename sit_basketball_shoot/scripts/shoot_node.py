import rospy

from shoot_controller import ShootController
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from sit_basketball_shoot_srv.srv import *


class ShootNode:
    """
    电磁炮ROS节点包装
    """
    def __init__(self, shoot_controller: ShootController):
        self.__shoot_controller = shoot_controller
        rospy.Service('%s/Shoot' % rospy.names.get_name(), Empty, self.__shoot_handle)
        rospy.Service('%s/Zero' % rospy.names.get_name(), Empty, self.__zero_handle)
        rospy.Service('%s/Charge' % rospy.names.get_name(), ShootCharge, self.__charge_handle)
        rospy.Service('%s/Status' % rospy.names.get_name(), ShootStatus, self.__status_handle)

    def __shoot_handle(self, request: EmptyRequest):
        """
        发射
        :return:
        """
        self.__shoot_controller.shoot()
        return EmptyResponse()

    def __zero_handle(self, request: EmptyRequest):
        """
        电磁炮零状态
        :param request:
        :return:
        """
        self.__shoot_controller.zero()
        return EmptyResponse()

    def __charge_handle(self, request: ShootChargeRequest):
        """
        充电
        :return:
        """
        self.__shoot_controller.charge(request.charge_time)
        return ShootChargeResponse()

    def __status_handle(self, request: ShootStatusRequest):
        resp = ShootStatusResponse()
        resp.status = self.__shoot_controller.status.value
        return resp
