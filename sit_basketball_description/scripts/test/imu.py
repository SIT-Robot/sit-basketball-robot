import sys

from roslaunch.core import Node


import rospy
from sensor_msgs.msg import Imu

from geometry_msgs.msg import Twist
import sys
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
rospy.init_node('test', sys.argv)

roll = 0
pitch = 0
yaw = 0
curr_vx = 0

def odom_cb(odom_msg: Odometry):
    global curr_vx
    curr_vx = odom_msg.twist.twist.linear.x

def imu_cb(imu_msg: Imu):
    global roll, pitch, yaw
    ori = imu_msg.orientation
    roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])


rospy.Subscriber('/imu_raw', Imu, imu_cb, queue_size=10)
rospy.Subscriber('/odom',Odometry,odom_cb,queue_size=10)
spd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


def set_vx(vx: float):
    twist = Twist()
    twist.linear.x = vx
    # twist.angular.z = 0.5
    spd_pub.publish(twist)


def timer(t):
    set_vx(pitch * 0.9 + curr_vx)


rospy.Timer(rospy.Duration.from_sec(0.02), timer)

try:
    rospy.spin()
except KeyboardInterrupt:
    pass
