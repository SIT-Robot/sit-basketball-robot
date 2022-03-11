from geometry_msgs.msg import Twist, TwistStamped
import rospy


class Chassis:
    """
    底盘运动控制
    """

    def __init__(self) -> None:
        # 期望速度
        self.expected_vx = 0
        self.expected_vy = 0
        self.expected_vw = 0

        # 真实速度
        self.real_vx = 0
        self.real_vy = 0
        self.real_vw = 0

        # 下发控制指令
        self._pub = rospy.Publisher('/cmd_vel', Twist,
                                    queue_size=10)

        # 接收真实速度
        self._sub = rospy.Subscriber('/real_speed', TwistStamped,
                                     callback=self.real_speed_cb,
                                     queue_size=10)

        # 开启定时器，定时更新底盘速度
        self._timer = rospy.Timer(rospy.Duration.from_sec(0.05), self.cmd_vel_update_handle)

    def cmd_vel_update_handle(self,t):
        """
        ROS速度更新定时器处理函数
        """
        twist = Twist()
        twist.linear.x = self.expected_vx
        twist.linear.y = self.expected_vy
        twist.angular.z = self.expected_vw
        self._pub.publish(twist)

    def real_speed_cb(self, real_speed: TwistStamped):
        """
        ROS订阅实际速度的订阅者回调函数
        """
        self.real_vx = real_speed.twist.linear.x
        self.real_vy = real_speed.twist.linear.y
        self.real_vw = real_speed.twist.angular.z
