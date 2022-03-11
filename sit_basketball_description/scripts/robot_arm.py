import gazebo_plugin
import links
import robot_links
from constants import color_consts
from urdf_parser_py.urdf import *
import math
from robot import Robot, JointType

from constants.robot_params import *


def add_arm_joint_transmission(robot: Robot,
                               joint: Joint):
    xml = f"""
    <transmission name="{joint.name}_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="{joint.name}">
            <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name="{joint.name}_motor">
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
    """
    robot.add_transmission_xml(xml)


def create_screw_motor(robot: Robot,
                       parent_link: Link,
                       origin: Pose):
    """
    创建丝杆电机
    :param robot:机器人
    :param parent_link: 父节点
    :param origin: 坐标变换
    :return: 丝杆电机Link
    """
    link, gazebo = links.create_box_link(link_name='arm_screw_motor',
                                         x_length=arm_screw_motor_length,
                                         y_length=arm_screw_motor_width,
                                         z_length=arm_screw_motor_height,
                                         weight=arm_screw_motor_weight,
                                         material=color_consts.black_color)
    robot.add_link(link)
    robot.add_gazebo(gazebo)

    joint = robot.joint_links(parent=parent_link,
                              child=link,
                              joint_type=JointType.fixed,
                              origin=origin)
    return link


def create_screw_slider_odom(robot: Robot,
                             screw: Link):
    slider_odom = Link(name='arm_slider_odom')
    robot.add_link(slider_odom)

    robot.joint_links(parent=screw,
                      child=slider_odom,
                      joint_type=JointType.fixed,
                      origin=Pose(
                          xyz=(0, 0, arm_screw_slider_height / 2)
                      ))
    return slider_odom


def create_screw_slider(robot: Robot,
                        slider_odom: Link):
    slider, gazebo = links.create_box_link(
        link_name='arm_slider',
        x_length=arm_screw_slider_length,
        y_length=arm_screw_slider_width,
        z_length=arm_screw_slider_height,
        weight=arm_screw_slider_weight,
        material=color_consts.black_color,
    )

    robot.add_link(slider)
    robot.add_gazebo(gazebo)
    joint = robot.joint_links(parent=slider_odom,
                              child=slider,
                              joint_type=JointType.prismatic,
                              # TODO
                              # joint_type=JointType.fixed,
                              axis=(0, 0, 1),
                              limit=JointLimit(
                                  effort=300,
                                  velocity=0.03,
                                  lower=0,
                                  upper=arm_screw_length - arm_screw_slider_height))

    add_arm_joint_transmission(robot, joint)
    return slider


def create_arm(robot: Robot,
               slider: Link):
    geometry = Cylinder(
        radius=0.02,
        length=arm_length)
    origin = Pose(
        xyz=(arm_length / 2, 0, 0),
        rpy=(0, math.pi / 2, 0)
    )
    link = Link(name='robot_arm',
                visual=Visual(
                    geometry=geometry,
                    material=color_consts.metal_color.rviz,
                    origin=origin
                ),
                collision=Collision(
                    geometry=geometry,
                    origin=origin
                ))

    robot.add_link(link)

    joint = robot.joint_links(
        parent=slider,
        child=link,
        joint_type=JointType.revolute,
        # TODO
        # joint_type=JointType.fixed,
        origin=Pose(
            xyz=(arm_screw_slider_length / 2, 0, 0)
        ),
        axis=(0, -1, 0),
        limit=JointLimit(
            effort=300,
            velocity=0.01,
            lower=0,
            upper=math.pi / 6,
        )
    )
    robot.add_gazebo(links.create_gazebo_material_label(link.name, color_consts.metal_color))

    add_arm_joint_transmission(robot, joint)
    return link


def create_claw_base(robot: Robot,
                     arm: Link):
    claw_base, gazebo = links.create_box_link(
        link_name='claw_base',
        x_length=claw_base_length,
        y_length=claw_base_width,
        z_length=claw_base_height,
        weight=0.01,
        material=color_consts.black_color
    )
    robot.add_link(claw_base)
    robot.add_gazebo(gazebo)

    joint = robot.joint_links(parent=arm,
                              child=claw_base,
                              joint_type=JointType.revolute,
                              # TODO
                              # joint_type=JointType.fixed,
                              axis=(0, 1, 0),
                              limit=JointLimit(
                                  effort=300,
                                  velocity=0.01,
                                  lower=0,
                                  upper=math.pi / 6,
                              ),
                              origin=Pose(
                                  xyz=(
                                      arm_length + claw_base_length / 2,
                                      0,
                                      0
                                  )
                              ))

    add_arm_joint_transmission(robot, joint)
    return claw_base


def create_claw(robot: Robot,
                claw_base: Link):
    grasping_frame = Link(name='grasping_frame')
    robot.add_link(grasping_frame)
    robot.joint_links(parent=claw_base,
                      child=grasping_frame,
                      joint_type=JointType.fixed,
                      origin=Pose(
                          xyz=(claw_base_length / 2, 0, 0)
                      ))

    def create_finger(prefix: str) -> Link:
        geometry = Box(size=(finger_length,
                             finger_width,
                             finger_height))
        origin = Pose(
            xyz=(finger_length / 2, 0, 0),
        )
        link = Link(name=f'{prefix}_finger',
                    visual=Visual(
                        geometry=geometry,
                        material=color_consts.metal_color.rviz,
                        origin=origin
                    ),
                    collision=Collision(
                        geometry=geometry,
                        origin=origin
                    ))
        robot.add_link(link)
        robot.add_gazebo(links.create_gazebo_material_label(link.name, color_consts.metal_color))
        return link

    left_finger = create_finger('left')
    right_finger = create_finger('right')

    robot.joint_links(
        parent=claw_base,
        child=left_finger,
        joint_type=JointType.fixed,
        origin=Pose(
            xyz=(claw_base_length / 2, claw_base_width / 2, 0)
        )
    )
    right_finger_joint = robot.joint_links(
        parent=claw_base,
        child=right_finger,
        joint_type=JointType.revolute,
        # TODO
        # joint_type=JointType.fixed,
        origin=Pose(
            xyz=(claw_base_length / 2, -claw_base_width / 2, 0)
        ),
        axis=(0, 0, 1),
        limit=JointLimit(
            effort=300,
            velocity=0.01,
            lower=0,
            upper=math.pi / 6,
        )
    )

    add_arm_joint_transmission(robot, right_finger_joint)

    return right_finger


def add_robot_arm(robot: Robot,
                  parent_link: Link):
    # 创建机械臂的底座
    arm_base_link = create_screw_motor(
        robot=robot,
        parent_link=parent_link,
        origin=Pose(
            xyz=(arm_screw_motor2center_distance,
                 0,
                 (arm_screw_motor_height + arm_base_link_height) / 2),
            rpy=(0, 0, 0)
        ))

    # 创建丝杆
    screw = robot_links.create_metal_link(
        robot=robot,
        parent_link=arm_base_link,
        name_prefix='arm_screw',
        length=arm_screw_length,
        origin=Pose(
            xyz=(0, 0, arm_screw_motor_height / 2)
        )
    )

    # 创建滑动台里程坐标系
    slider_odom = create_screw_slider_odom(robot, screw)

    # 创建滑动台
    slider = create_screw_slider(robot, slider_odom)

    # 创建手臂
    arm = create_arm(robot, slider)

    # 创建夹抓底座
    claw_base = create_claw_base(robot, arm)

    # create claw
    create_claw(robot, claw_base)

    gazebo_plugin.add_ros_control(robot)
