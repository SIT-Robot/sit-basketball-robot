import rospy
from rospy.service import ServiceException
from sit_protocol_msgs.srv import RequestDataFrame, RequestDataFrameRequest, RequestDataFrameResponse
import enum


class ShootStatusEnum(enum.IntEnum):
    """表示电磁炮状态的枚举类
    """
    # 未就绪的
    UNREADY = 0x00

    # 已就绪的
    READY = 0x01


class ShootController:
    def __init__(self):
        # 串口服务器
        self._serial_server_proxy = rospy.ServiceProxy(
            '/protocol_forwarder/RequestDataFrame', RequestDataFrame)
        rospy.loginfo('等待连接串口服务器')
        self._serial_server_proxy.wait_for_service()
        rospy.loginfo('串口服务器连接成功')

    def zero(self):
        req = RequestDataFrameRequest()
        req.request.address - 0x01
        req.request.cmd = 0xB3
        req.waitCmd = 0x3B
        try:
            # 请求响应，无需响应数据
            self._serial_server_proxy.call(req)

        except ServiceException as e:
            rospy.logerr('请求超时：'+str(e))

    def charge(self, charge_time: int):
        """电磁炮充电
            注意，该函数不会阻塞，请手动阻塞
        Args:
            charge_time (int): 充电时间,0到65535

        Returns:
            [bool]: true表示充电成功，
            false表示之前已经充电完毕，需要放电后重新充电
        """
        req = RequestDataFrameRequest()
        req.request.address = 0x01
        req.request.cmd = 0x05
        req.request.data = [(charge_time >> 8) & 0xff, charge_time & 0xff]
        req.waitCmd = 0x50
        try:
            # 请求响应
            self._serial_server_proxy.call(req)
            rospy.loginfo('充电指令已发出(时间:%d ms)',charge_time)

        except ServiceException as e:
            rospy.logerr('请求超时：'+str(e))

    def shoot(self):
        req = RequestDataFrameRequest()
        req.request.address = 0x01
        req.request.cmd = 0x06
        req.waitCmd = 0x60
        try:
            # 请求响应，无需响应数据
            self._serial_server_proxy.call(req)
            rospy.loginfo('电磁炮已发射')
        except ServiceException as e:
            rospy.logerr('请求超时：'+str(e))

    @property
    def status(self) -> ShootStatusEnum:
        """获取电磁炮状态
        """
        req = RequestDataFrameRequest()
        req.request.address = 0x01
        req.request.cmd = 0x07
        req.waitCmd = 0x70
        try:
            # 请求响应
            resp: RequestDataFrameResponse = self._serial_server_proxy.call(req)

            # 获得电磁炮状态码
            status_code: int = resp.response.data[0]

            # 按枚举返回
            return {
                0x00: ShootStatusEnum.UNREADY,
                0x01: ShootStatusEnum.READY,
            }[status_code]

        except ServiceException as e:
            rospy.logerr('请求超时：'+str(e))
