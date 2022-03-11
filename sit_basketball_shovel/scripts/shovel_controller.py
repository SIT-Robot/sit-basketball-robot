from enum import IntEnum
import rospy
from rospy.service import ServiceException
from sit_protocol_msgs.msg import CmdDataFrame
from sit_protocol_msgs.srv import RequestDataFrameRequest,RequestDataFrame


class ControlType(IntEnum):
    """
    控制类型的枚举
    """
    STOP = 0x00
    UP = 0x01
    DOWN = 0x02


class ShovelController:
    """
    起落架控制器
    """

    def __init__(self):
        # 软串口服务器调用代理
        self.req_proxy = rospy.ServiceProxy(
            '/protocol_forwarder/RequestDataFrame', RequestDataFrame)

        # 正在等待服务可用
        rospy.loginfo('等待串口服务器可用')
        self.req_proxy.wait_for_service()
        rospy.loginfo('串口服务器链接成功')

    def send_shovel_cmd(self, control_type: ControlType):
        """
        发送起落架控制指令
        """
        req = RequestDataFrameRequest()
        req.request.address = 0x01
        req.request.cmd = 0x08
        req.request.data = [control_type.value]
        req.waitCmd = 0x80
        try:
            self.req_proxy.call(req)
        except ServiceException as e:
            rospy.logerr('请求超时: '+str(e))


