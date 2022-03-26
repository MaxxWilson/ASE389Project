
import os

import roboticstoolbox as rtb
from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.tools import URDF
import xml.etree.ElementTree as ET
import xacro

from ament_index_python import get_package_share_directory

class Atlas(ERobot):
    def __init__(self) -> None:
        links, name, urdf_string, urdf_filepath = self.URDF_read("atlas/atlas_robot_toolbox.urdf")

        super().__init__(
            links,
            name=name,
            manufacturer="Boston Dynamics",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

robot = Atlas()
print(robot)
