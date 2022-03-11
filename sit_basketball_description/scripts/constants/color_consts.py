from typing import NamedTuple
from urdf_parser_py.urdf import Material, Color
import urdf_parser_py.xml_reflection.core as xmlr
from urdf_parser_py.urdf import ET


def get_rgba_color(red: int, green: int, blue: int, alpha: float = 1.0):
    return Color(red / 255, green / 255, blue / 255, alpha)


class ColorMaterial(NamedTuple):
    rviz: Material
    gazebo: str


black_color = ColorMaterial(rviz=Material(name='black',
                                          color=get_rgba_color(0, 0, 0, 0.8)),
                            gazebo='<material>Gazebo/Black</material>')

metal_color = ColorMaterial(rviz=Material(name='metal',
                                          color=get_rgba_color(103, 121, 134, 0.8)),
                            gazebo='<material>Gazebo/Metal</material>')

blue_color = ColorMaterial(rviz=Material(name='blue',
                                          color=get_rgba_color(0, 0, 204, 0.8)),
                            gazebo='<material>Gazebo/Blue</material>')
