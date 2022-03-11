#! /bin/python3
import rospy
from nav_msgs.msg import OccupancyGrid

from draw_map import get_blank_map, get_visual_map

rospy.init_node('basketball_map_server')

visual_map = get_visual_map().to_ros_map()
blank_map = get_blank_map().to_ros_map()

# 可视化地图发布
visual_map_pub = rospy.Publisher('/visual_map',
                OccupancyGrid,
                queue_size=1,
                latch=True)
visual_map_pub.publish(visual_map)

# 用于导航的空白地图发布
blank_map_pub = rospy.Publisher('/map',
                OccupancyGrid,
                queue_size=1,
                latch=True)
blank_map_pub.publish(blank_map)

try:
    rospy.spin()
except KeyboardInterrupt:
    print('退出地图服务器')
