from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib
import threading
from re import A, M
import rospy

rospy.init_node('test')


def spin_thread():
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
# threading.Thread(target=spin_thread)


client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)


def move(x, y, th):

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_footprint'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    from tf.transformations import quaternion_from_euler
    a = quaternion_from_euler(0, 0, th)
    # goal.target_pose.pose.orientation.x = a[0]
    # goal.target_pose.pose.orientation.y = a[1]
    # goal.target_pose.pose.orientation.z = a[2]
    goal.target_pose.pose.orientation.w = 1
    # print(goal)
    client.send_goal(goal)
    # client.send_goal_and_wait(goal, execute_timeout=rospy.Duration.from_sec(2))


print('start move')

target_id = 0
target_pose = [
    (1, 0, 0),
    (0,1,0),
    (-1, 0, 0),
    (0,-1,0)
]


def timer(t):
    global target_id
    target_id = (target_id + 1) % len(target_pose)
    pose = target_pose[target_id]
    print(target_id)
    move(pose[0], pose[1], pose[2])


rospy.Timer(rospy.Duration.from_sec(10), timer)

spin_thread()
