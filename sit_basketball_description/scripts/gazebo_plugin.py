from urdf_parser_py.urdf import *
from constants.robot_params import *
from robot import Robot
import math


def add_drive_controller(robot: Robot,
                         left_joint: Joint,
                         right_joint: Joint,
                         root_frame: str):
    xml_str = f"""
    <!-- 差速控制器 -->
<gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
        <commandTopic>cmd_vel</commandTopic>
        <odometryTopic>odom</odometryTopic>
        <odometryFrame>odom</odometryFrame>
        <odometrySource>world</odometrySource>
        <publishOdomTF>true</publishOdomTF>
        <robotBaseFrame>{root_frame}</robotBaseFrame>
        <publishWheelTF>false</publishWheelTF>
        <publishTf>true</publishTf>
        <publishWheelJointState>true</publishWheelJointState>
        <legacyMode>false</legacyMode>
        <updateRate>30</updateRate>
        <leftJoint>{left_joint.name}</leftJoint>
        <rightJoint>{right_joint.name}</rightJoint>
        <wheelSeparation>{wheel2base_center_distance * 2}</wheelSeparation>
        <wheelDiameter>{drive_wheel_radius * 2}</wheelDiameter>
        <wheelAcceleration>1</wheelAcceleration>
        <wheelTorque>10</wheelTorque>
        <rosDebugLevel>na</rosDebugLevel>
    </plugin>
</gazebo>
    """

    robot.add_gazebo_xml(xml_str)


def add_ros_control(robot: Robot):
    xml_str = """
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace>/</robotNamespace>
        </plugin>
    </gazebo>
    """
    robot.add_gazebo_xml(xml_str)


def add_laser_sensor(robot: Robot,
                     reference_link: Link):
    gazebo_xml_str = f"""
    <!-- hokuyo -->
    <gazebo reference="{reference_link.name}">
        <sensor type="ray" name="head_hokuyo_sensor">
            <pose>0 0 0 0 0 0</pose>
            <visualize>false</visualize>
            <update_rate>40</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>720</samples>
                        <resolution>1</resolution>
                        <min_angle>{-math.pi / 2}</min_angle>
                        <max_angle>{math.pi / 2}</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>0.10</min>
                    <max>30.0</max>
                    <resolution>0.01</resolution>
                </range>
                <noise>
                    <type>gaussian</type>
                    <!-- Noise parameters based on published spec for Hokuyo laser
                         achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
                         stddev of 0.01m will put 99.7% of samples within 0.03m of the true
                         reading. -->
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </ray>
            <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
                <topicName>scan</topicName>
                <frameName>{reference_link.name}</frameName>
            </plugin>
        </sensor>
    </gazebo>
    """
    robot.add_gazebo_xml(gazebo_xml_str)


def add_imu_sensor(robot: Robot,
                   reference_link: Link):
    gazebo_xml_str = f"""
    <gazebo reference="{reference_link.name}">
        <gravity>true</gravity>
        <sensor name="imu_sensor" type="imu">
            <always_on>true</always_on>
            <update_rate>100</update_rate>
            <visualize>true</visualize>
            <topic>imu_raw</topic>
            <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
              <topicName>imu_raw</topicName>
              <bodyName>{reference_link.name}</bodyName>
              <updateRateHZ>10.0</updateRateHZ>
              <gaussianNoise>0.0</gaussianNoise>
              <xyzOffset>0 0 0</xyzOffset>
              <rpyOffset>0 0 0</rpyOffset>
              <frameName>{reference_link.name}</frameName>
              <initialOrientationAsReference>false</initialOrientationAsReference>
            </plugin>
            <pose>0 0 0 0 0 0</pose>
        </sensor>
    </gazebo>
    """
    robot.add_gazebo_xml(gazebo_xml_str)


def add_rgbd_camera_sensor(robot: Robot,
                           reference_link: Link):
    xml = f"""
    <gazebo reference="{reference_link.name}">
      <sensor type="depth" name="camera">
        <always_on>true</always_on>
        <update_rate>20.0</update_rate>
        <camera>
          <horizontal_fov>{60.0 * math.pi / 180.0}</horizontal_fov>
          <image>
            <format>R8G8B8</format>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.05</near>
            <far>8.0</far>
          </clip>
        </camera>
        <plugin name="kinect_camera_controller" filename="libgazebo_ros_openni_kinect.so">
          <cameraName>camera</cameraName>
          <alwaysOn>true</alwaysOn>
          <updateRate>10</updateRate>
          <imageTopicName>rgb/image_raw</imageTopicName>
          <depthImageTopicName>depth/image_raw</depthImageTopicName>
          <pointCloudTopicName>depth/points</pointCloudTopicName>
          <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
          <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
          <frameName>pointcloud</frameName>
          <baseline>0.1</baseline>
          <distortion_k1>0.0</distortion_k1>
          <distortion_k2>0.0</distortion_k2>
          <distortion_k3>0.0</distortion_k3>
          <distortion_t1>0.0</distortion_t1>
          <distortion_t2>0.0</distortion_t2>
          <pointCloudCutoff>0.4</pointCloudCutoff>
        </plugin>
      </sensor>
    </gazebo>
    """
    robot.add_gazebo_xml(xml)
