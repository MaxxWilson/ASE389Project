import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import OrderedDict

import pinocchio as pin

from pnc.robot_system.pinocchio_robot_system import PinocchioRobotSystem

class InteractiveRVIZWrapper(Node):
    def __init__(self, name, robot):
        super().__init__(name)

        self._robot = robot

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10)

    def joint_state_callback(self, msg):
        print(20 * "-")
        for i in range(len(msg.name)):
            print("Name: " + str(msg.name[i]))
            print("Position: " + str(msg.position[i]))
            print()

def main(args=None):
    rclpy.init(args=args)
    robot = PinocchioRobotSystem(
                cwd + "/robot_model/atlas/atlas.urdf",
                cwd + "/robot_model/atlas", False)
    node = InteractiveRVIZWrapper("interactive_rviz_wrapper", robot)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()