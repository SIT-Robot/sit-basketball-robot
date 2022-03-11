from urdf_parser_py.urdf import *
import inertial_common
from constants.color_consts import ColorMaterial
from typing import *
from urdf_parser_py.xml_reflection.core import Element, ET


def create_gazebo_material_label(link_name: str,
                                 material: ColorMaterial) -> Element:
    return ET.fromstring(f"""
    <gazebo reference="{link_name}">{material.gazebo}</gazebo>
    """)


def create_cylinder_link(link_name: str,
                         radius: float,
                         length: float,
                         weight: float,
                         material: ColorMaterial) -> Tuple[Link, Element]:
    geometry = Cylinder(radius=radius,
                        length=length, )

    link_label = Link(name=link_name,
                      visual=Visual(geometry=geometry,
                                    material=material.rviz),
                      collision=Collision(geometry=geometry),
                      inertial=inertial_common.cylinder_inertial_matrix(weight, radius, length))
    gazebo_label = create_gazebo_material_label(link_name, material)
    return link_label, gazebo_label


def create_box_link(link_name: str,
                    x_length: float,
                    y_length: float,
                    z_length: float,
                    weight: float,
                    material: ColorMaterial) -> Tuple[Link, Element]:
    geometry = Box(size=(x_length, y_length, z_length))
    link_label = Link(name=link_name,
                      visual=Visual(geometry=geometry,
                                    material=material.rviz),
                      collision=Collision(geometry=geometry),
                      inertial=inertial_common.box_inertial_matrix(weight, x_length, y_length, z_length))
    gazebo_label = create_gazebo_material_label(link_name, material)
    return link_label, gazebo_label


def create_sphere_link(link_name: str,
                       radius: float,
                       weight: float,
                       material: ColorMaterial) -> Tuple[Link, Element]:
    geometry = Sphere(radius)

    link_label = Link(
        name=link_name,
        visual=Visual(geometry=geometry,
                      material=material.rviz),
        collision=Collision(geometry=geometry),
        inertial=inertial_common.sphere_inertial_matrix(weight, radius)
    )
    gazebo_label = create_gazebo_material_label(link_name, material)
    return link_label, gazebo_label
