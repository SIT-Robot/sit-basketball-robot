from urdf_parser_py.urdf import *

robot = Robot.from_xml_string("""
<robot name="Hello">

</robot>
""")

print(robot.to_xml_string())
