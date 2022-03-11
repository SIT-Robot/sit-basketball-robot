from enum import Enum

from urdf_parser_py import urdf
from urdf_parser_py.urdf import Link, Joint, Transmission
from urdf_parser_py.xml_reflection.core import Element
from urdf_parser_py.urdf import *


class JointType(Enum):
    unknown = 'unknown'
    revolute = 'revolute'
    continuous = 'continuous'
    prismatic = 'prismatic'
    floating = 'floating'
    planar = 'planar'
    fixed = 'fixed'


class Robot(urdf.Robot):
    def __init__(self, name: str):
        super(Robot, self).__init__(name)

    def add_gazebo(self, gazebo_element: Element):
        self.add_aggregate('gazebo', gazebo_element)

    def add_gazebo_xml(self, gazebo_xml_string: str):
        self.add_gazebo(urdf.ET.fromstring(gazebo_xml_string))

    def add_material(self, material: Material):
        self.add_aggregate('material', material)

    def add_material_xml(self, material_xml_string: str) -> Material:
        material = Material.from_xml_string(material_xml_string)
        self.add_material(material)
        return material

    def add_joint_xml(self, joint_xml: str) -> Joint:
        joint = Joint.from_xml_string(joint_xml)
        self.add_joint(joint)
        return joint

    def add_link_xml(self, link_xml: str) -> Link:
        link = Link.from_xml_string(link_xml)
        self.add_link(link)
        return link

    def add_transmission(self, transmission: urdf.Transmission):
        self.add_aggregate('transmission', transmission)

    def __str__(self):
        return self.to_xml_string()

    def joint_links(self, parent: Link, child: Link, joint_type: JointType,
                    name: str = None,
                    axis=None, origin=None,
                    limit=None, dynamics=None, safety_controller=None,
                    calibration=None, mimic=None) -> Joint:
        if name is None:
            name = '%s_to_%s_joint' % (parent.name, child.name)

        joint = Joint(name=name,
                      parent=parent.name,
                      child=child.name,
                      joint_type=joint_type.value,
                      axis=axis, origin=origin,
                      limit=limit, dynamics=dynamics, safety_controller=safety_controller,
                      calibration=calibration, mimic=mimic)
        self.add_joint(joint)
        return joint

    def add_transmission_xml(self, xml: str):
        self.add_transmission(Transmission.from_xml_string(xml))

    def joint_trans(self, joint: Joint) -> Transmission:
        trans = Transmission.from_xml_string(f"""
            <transmission name="{joint.name}_trans">
                <type>transmission_interface/SimpleTransmission</type>
                <joint name="{joint.name}">
                    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
                </joint>
                <actuator name="{joint.name}_motor">
                    <mechanicalReduction>1</mechanicalReduction>
                </actuator>
            </transmission>
        """)

        self.add_transmission(trans)
        return trans
