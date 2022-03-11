import sys

import rospy

import shoot_controller

if __name__ == '__main__':
    rospy.init_node('shoot_node', sys.argv)

    sc = shoot_controller.ShootController()

    print('准备放电')
    sc.shoot()
    for i in range(5, 0, -1):
        rospy.sleep(1)
        print(i)

    print('充电前：', sc.status)

    a = 1
    sc.charge(int(a * 1000))
    for i in range(1, int(1 + a)):
        rospy.sleep(1)
        print(i)
    print('充电后：', sc.status)

    print('准备发射')
    rospy.sleep(1)
    sc.shoot()  # 发射
    rospy.sleep(a)  # 等一秒
    sc.zero()

    try:
        rospy.spin()
    except KeyboardInterrupt as e:
        rospy.loginfo(e)
