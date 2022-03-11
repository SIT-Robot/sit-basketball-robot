from sit_basketball_shovel_srv.srv import ShovelControl, ShovelControlRequest, ShovelControlResponse
from shovel_controller import ShovelController, ControlType
from std_msgs.msg import UInt8
import rospy


class ShovelNode:
    """
    起落架节点
    """

    def __init__(self, shovel_controller: ShovelController):
        self.shovel_controller = shovel_controller
        self.shovel_server = rospy.Service(
            'basketball_shovel/ShovelControl', ShovelControl, self.shovel_control_handle)
        self.shovel_subscriber = rospy.Subscriber(
            'basketball_shovel/shovel', UInt8, callback=self.shovel_sub_cb, queue_size=10)

    def control_shovel(self, control_type: int):
        if control_type == 0x00:
            # 停止
            self.shovel_controller.send_shovel_cmd(ControlType.STOP)
            rospy.loginfo('起落架已停止')

        elif control_type == 0x01:
            # 上升
            self.shovel_controller.send_shovel_cmd(ControlType.UP)
            rospy.loginfo('起落架上升')

        elif control_type == 0x02:
            # 下降
            self.shovel_controller.send_shovel_cmd(ControlType.DOWN)
            rospy.loginfo('起落架下降')

        else:
            # 未知指令
            rospy.logerr('未知的起落架控制指令' + hex(control_type))

    def shovel_sub_cb(self, control_type: UInt8):
        self.control_shovel(control_type.data)

    def shovel_control_handle(self, request: ShovelControlRequest):
        self.control_shovel(request.control_type)
        return ShovelControlResponse()
