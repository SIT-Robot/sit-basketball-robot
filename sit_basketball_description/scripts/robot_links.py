from urdf_parser_py.urdf import *

import inertial_common
from robot import Robot, JointType
from constants.robot_params import *
from constants import color_consts, mesh_files
import links
import math
from typing import Tuple
from robot_arm import add_robot_arm

def create_footprint_link(robot: Robot) -> Link:
    link = Link(name='base_footprint')
    robot.add_link(link)
    return link


def create_base_link(robot: Robot, footprint_link: Link = None) -> Link:
    base_link, base_link_gazebo = links.create_cylinder_link(
        link_name='base_link',
        radius=base_link_radius,
        length=base_link_height,
        weight=base_link_weight,
        material=color_consts.black_color)
    robot.add_link(base_link)
    robot.add_gazebo(base_link_gazebo)

    if footprint_link is not None:
        robot.joint_links(
            parent=footprint_link,
            child=base_link,
            joint_type=JointType.fixed,
            origin=Pose(xyz=(0, 0, base2footprint_distance)))
    return base_link


def create_laser_link(robot: Robot, base_link: Link) -> Link:
    visual_geometry = Mesh(filename=mesh_files.hokuyo_mesh,
                           scale=(1, 1, 1))
    collision_geometry = Box(size=(laser_length,
                                   laser_width,
                                   laser_height,))

    laser_link = Link(name='laser_link',
                      visual=Visual(geometry=visual_geometry, ),
                      collision=Collision(geometry=collision_geometry),
                      inertial=inertial_common.box_inertial_matrix(laser_weight,
                                                                   laser_length,
                                                                   laser_width,
                                                                   laser_height, ))
    robot.add_link(laser_link)
    robot.joint_links(
        parent=base_link,
        child=laser_link,
        joint_type=JointType.fixed,
        origin=Pose(xyz=(laser2base_center_distance,
                         0,
                         (laser_height + base_link_height) / 2)))

    return laser_link


def create_imu_link(robot: Robot, base_link: Link) -> Link:
    imu_link, imu_gazebo = links.create_box_link(
        link_name='imu_link',
        x_length=imu_length,
        y_length=imu_width,
        z_length=imu_height,
        weight=imu_weight,
        material=color_consts.metal_color,
    )
    robot.add_link(imu_link)
    robot.add_gazebo(imu_gazebo)

    robot.joint_links(
        parent=base_link,
        child=imu_link,
        joint_type=JointType.fixed,
        origin=Pose(xyz=(0,
                         0,
                         -(imu_height + base_link_height) / 2)))
    return imu_link


def create_metal_link(robot: Robot,
                      parent_link: Link,
                      name_prefix: str,
                      length: float,
                      origin: Pose = None) -> Link:
    radius = 0.01
    weight_per_m = 1

    geometry = Cylinder(radius=radius,
                        length=length, )

    link_label = Link(name=f'{name_prefix}_metal_link',
                      visual=Visual(geometry=geometry,
                                    origin=Pose(xyz=(0, 0, length / 2)),
                                    material=color_consts.metal_color.rviz),
                      collision=Collision(geometry=geometry,
                                          origin=Pose(xyz=(0, 0, length / 2))),
                      inertial=inertial_common.cylinder_inertial_matrix(weight_per_m * length, radius, length))
    gazebo_label = links.create_gazebo_material_label(link_label.name, color_consts.metal_color)

    robot.add_link(link_label)
    robot.add_gazebo(gazebo_label)

    robot.joint_links(
        parent=parent_link,
        child=link_label,
        joint_type=JointType.fixed,
        origin=origin,
    )
    return link_label


def create_battery_base_link(robot: Robot, metal_link: Link) -> Link:
    """
    创建电池底盘
    :param robot:机器人
    :param metal_link:机器人底盘
    :return:电池底盘
    """
    battery_base_link, battery_base_link_gazebo = links.create_cylinder_link(
        link_name='battery_base_link',
        radius=battery_base_link_radius,
        length=battery_base_link_height,
        weight=battery_base_link_weight,
        material=color_consts.metal_color)
    robot.add_link(battery_base_link)
    robot.add_gazebo(battery_base_link_gazebo)
    robot.joint_links(
        parent=metal_link,
        child=battery_base_link,
        joint_type=JointType.fixed,
        origin=Pose(
            xyz=(0,
                 0,
                 battery2base_distance)
        )
    )
    return battery_base_link


def create_battery_link(robot: Robot, battery_base_link: Link, prefix: str) -> Link:
    battery_link, battery_gazebo = links.create_box_link(
        link_name=f'{prefix}_battery_link',
        x_length=battery_length,
        y_length=battery_width,
        z_length=battery_height,
        weight=battery_weight,
        material=color_consts.blue_color,
    )
    robot.add_link(battery_link)
    robot.add_gazebo(battery_gazebo)

    robot.joint_links(
        parent=battery_base_link,
        child=battery_link,
        joint_type=JointType.fixed,
        origin=Pose(xyz=(
            0,
            ((battery_width + battery_distance) / 2) * (1 if prefix == 'left' else -1),
            (battery_height + battery_base_link_height) / 2
        ))
    )
    return battery_link


def create_arm_base_link(robot: Robot, metal_link: Link) -> Link:
    """
    创建电池底盘
    :param robot:机器人
    :param metal_link:机器人底盘
    :return:电池底盘
    """
    arm_base_link, arm_base_link_gazebo = links.create_cylinder_link(
        link_name='arm_base_link',
        radius=arm_base_link_radius,
        length=arm_base_link_height,
        weight=arm_base_link_weight,
        material=color_consts.metal_color)
    robot.add_link(arm_base_link)
    robot.add_gazebo(arm_base_link_gazebo)
    robot.joint_links(
        parent=metal_link,
        child=arm_base_link,
        joint_type=JointType.fixed,
        origin=Pose(
            xyz=(0,
                 0,
                 arm2battery_distance)
        )
    )
    return arm_base_link


def create_camera_base_link(robot: Robot, metal_link: Link) -> Link:
    """
    创建相机底盘
    :param robot:机器人
    :param metal_link:相机底盘连接杆
    :return:相机底盘
    """
    camera_base_link, camera_base_link_gazebo = links.create_box_link(
        link_name='camera_base_link',
        x_length=camera_base_link_length,
        y_length=camera_base_link_width,
        z_length=camera_base_link_height,
        weight=camera_base_link_weight,
        material=color_consts.black_color,
    )
    robot.add_link(camera_base_link)
    robot.add_gazebo(camera_base_link_gazebo)
    robot.joint_links(
        parent=metal_link,
        child=camera_base_link,
        joint_type=JointType.fixed,
        origin=Pose(
            xyz=(0,
                 0,
                 camera2arm_distance)
        )
    )
    return camera_base_link


def add_d435i_rgbd_camera_link(robot: Robot,
                               camera_base_link: Link) -> Link:
    robot.add_material_xml("""
    <material name="aluminum">
        <color rgba="0.5 0.5 0.5 0.8"/>
    </material>
    """)

    robot.add_material_xml("""
    <material name="plastic">
        <color rgba="0.1 0.1 0.1 0.8"/>
    </material>
    """)

    robot.add_joint_xml(f"""
    <joint name="camera_joint" type="fixed">
        <origin rpy="0 0 0" xyz="{camera_base_link_length / 2} 0 0.2"/>
        <parent link="{camera_base_link.name}"/>
        <child link="camera_bottom_screw_frame"/>
    </joint>
    """)

    robot.add_link_xml("""
    <link name="camera_bottom_screw_frame"/>
    """)

    robot.add_joint_xml("""
    <joint name="camera_link_joint" type="fixed">
        <origin rpy="0 0 0" xyz="0.010600000000000002 0.0175 0.0125"/>
        <parent link="camera_bottom_screw_frame"/>
        <child link="camera_link"/>
    </joint>
    """)

    camera_link = robot.add_link_xml("""
      <link name="camera_link">
    <visual>
       <!-- the mesh origin is at front plate in between the two infrared camera axes -->
       <origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0.0043 -0.0175 0"/>
       <geometry>
         <mesh filename="package://realsense2_description/meshes/d435.dae"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0 0 0" xyz="0 -0.0175 0"/>
       <geometry>
         <box size="0.02505 0.09 0.025"/>
       </geometry>
     </collision>
     <inertial>
       <!-- The following are not reliable values, and should not be used for modeling -->
       <mass value="0.072"/>
       <origin xyz="0 0 0"/>
       <inertia ixx="0.003881243" ixy="0.0" ixz="0.0" iyy="0.000498940" iyz="0.0" izz="0.003879257"/>
     </inertial>
    </link>
    """)
    return camera_link


def create_support_wheel(robot: Robot,
                         base_link: Link,
                         prefix: str):
    wheel_link, wheel_gazebo = links.create_sphere_link(
        link_name=f'{prefix}_support_wheel',
        radius=base2footprint_distance / 2 - 0.00001,
        weight=support_wheel_weight,
        material=color_consts.metal_color,
    )
    robot.add_link(wheel_link)
    robot.add_gazebo(wheel_gazebo)

    robot.joint_links(
        parent=base_link,
        child=wheel_link,
        joint_type=JointType.fixed,
        origin=Pose(
            xyz=((wheel2base_center_distance * (1 if prefix == 'front' else -1)),
                 0,
                 -(base_link_height + base2footprint_distance) / 2),
            rpy=(0, 0, 0)
        )
    )


def create_drive_wheel(robot: Robot,
                       base_link: Link,
                       prefix: str) -> Tuple[Link, Joint]:
    """
    创建驱动轮
    :param robot: 机器人
    :param base_link: 底盘
    :param prefix: 左(left)右(right)轮
    :return: 驱动轮
    """
    wheel_link, wheel_gazebo = links.create_cylinder_link(
        link_name=f'{prefix}_drive_wheel',
        radius=drive_wheel_radius,
        length=drive_wheel_height,
        weight=drive_wheel_weight,
        material=color_consts.metal_color,
    )
    robot.add_link(wheel_link)
    robot.add_gazebo(wheel_gazebo)

    __joint = robot.joint_links(
        parent=base_link,
        child=wheel_link,
        joint_type=JointType.continuous,
        axis=(0, 0, 1),
        origin=Pose(
            xyz=(0,
                 (wheel2base_center_distance * (1 if prefix == 'left' else -1)),
                 -(base_link_height / 2 +
                   base2footprint_distance -
                   drive_wheel_radius)),
            rpy=(-math.pi / 2, 0, 0)
        )
    )

    robot.joint_trans(__joint)
    return wheel_link, __joint
