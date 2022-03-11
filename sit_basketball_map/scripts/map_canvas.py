from PIL import Image, ImageDraw
from typing import List, Tuple
from nav_msgs.msg import OccupancyGrid
import rospy
import ctypes


class MapCanvas:
    def __init__(self,
                 resolution: float,
                 width: float,
                 height: float):
        """
        初始化一个地图绘制器
        :param resolution:分辨率(单位：毫米/像素)
        :param width:地图宽度(单位：米)
        :param height:地图高度(单位：米)
        """
        self.__width = width
        self.__height = height
        self.__img = Image.new(
            mode='L',
            size=(int(width / resolution),
                  int(height / resolution)),
            color=255)
        self.__resolution = resolution
        self.__drawer = ImageDraw.Draw(self.__img)

        self.line_width = 10.

        self.__current_x = 0.
        self.__current_y = 0.

    def convert_num(self, real_num: float) -> int:
        return int(real_num / self.__resolution)

    def convert_coord(self,
                      real_x: float,
                      real_y: float) -> Tuple[int, int]:
        return (self.convert_num(real_x),
                self.convert_num(real_y))

    def draw_start(self,
                   start_x: float,
                   start_y: float):
        self.__current_x = start_x
        self.__current_y = start_y

    def draw_move(self,
                  delta_x: float,
                  delta_y: float):
        self.__current_x += delta_x
        self.__current_y += delta_y

    def circle(self,
               radius: float,
               fill: bool = False):
        start_x = self.__current_x - radius
        start_y = self.__current_y - radius

        end_x = self.__current_x + radius
        end_y = self.__current_y + radius
        self.__drawer.ellipse(
            xy=[self.convert_coord(start_x, start_y),
                self.convert_coord(end_x, end_y)],
            width=self.convert_num(self.line_width),
            fill=0 if fill else None
        )

    def line_to(self,
                next_dx: float,
                next_dy: float):
        next_x = next_dx + self.__current_x
        next_y = next_dy + self.__current_y

        self.__drawer.line(
            xy=[self.convert_coord(self.__current_x, self.__current_y),
                self.convert_coord(next_x, next_y)],
            width=self.convert_num(self.line_width)
        )
        self.__current_x = next_x
        self.__current_y = next_y

    def serial_lines(self, deltas: List[Tuple[float, float]]):
        for delta in deltas:
            self.line_to(delta[0], delta[1])

    def rect(self, width: float, height: float):
        next_x = width + self.__current_x
        next_y = height + self.__current_y

        self.__drawer.rectangle(
            xy=[self.convert_coord(self.__current_x, self.__current_y),
                self.convert_coord(next_x, next_y)],
            width=self.convert_num(self.line_width)
        )

    def to_ros_map(self):
        og = OccupancyGrid()
        og.header.stamp = rospy.Time.now()
        og.header.frame_id = 'map'

        og.info.map_load_time = rospy.Time.now()
        og.info.resolution = self.__resolution
        og.info.width = self.__img.width
        og.info.height = self.__img.height

        og.info.origin.position.x = -self.__width / 2
        og.info.origin.position.y = -self.__height / 2

        og.info.origin.orientation.w = 1

        raw_data: bytes = self.__img.tobytes('raw')

        og.data = list(map(lambda x: ctypes.c_int8(x).value, raw_data))
        return og

    def show(self):
        import matplotlib.pyplot as plt
        plt.figure('test')
        plt.imshow(self.__img)
        plt.show()
