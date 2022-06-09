import os
import sys
from collections import OrderedDict

cwd = os.getcwd()
sys.path.append(cwd)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
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

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "atlas_markers",
            10)

        self.last_joint_state = None

    def joint_state_callback(self, msg):
        curr_joint_state = np.array(msg.position)
        if(self.last_joint_state is None or abs(sum(curr_joint_state - self.last_joint_state)) > 1e-2):
            self.last_joint_state = curr_joint_state

            base_joint_pos = [0, 0, 0] 
            base_joint_quat = [0, 0, 0, 1]
            base_joint_lin_vel = [0, 0, 0] 
            base_joint_ang_vel = [0, 0, 0]

            joint_pos = OrderedDict()
            joint_vel = OrderedDict()

            for i in range(len(msg.name)):
                joint_pos[msg.name[i]] = msg.position[i]
                
                if(len(msg.velocity) != 0):
                    joint_vel[msg.name[i]] = msg.velocity[i]
                else:
                    joint_vel[msg.name[i]] = 0

            self._robot.update_system(
                None,
                None,
                None,
                None,
                base_joint_pos,
                base_joint_quat,
                base_joint_lin_vel,
                base_joint_ang_vel,
                joint_pos,
                joint_vel,
                True)

            msg = MarkerArray()
            msg.markers.append(self.gen_com_marker())
            msg.markers += self.gen_inertia_markers()

            self.marker_publisher.publish(msg)

    def gen_inertia_markers(self):

        com = self._robot.get_com_pos()

        Ixx = self._robot._Ig[0, 0]
        Iyy = self._robot._Ig[1, 1]
        Izz = self._robot._Ig[2, 2]

        msgs = []
        nominal_inertia = 50
        inertias = [Ixx, Iyy, Izz]

        for i in range(3):
            msg = Marker()
            msg.header.frame_id = "pelvis"
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.position.x = com[0]
            msg.pose.position.y = com[1]
            msg.pose.position.z = com[2]

            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0

            msg.scale.x = 0.3
            msg.scale.y = 0.02
            msg.scale.z = 0.02
            msg.color.a = 1.0

            msg.ns = "atlas"
            msg.type = Marker.ARROW
            msg.action = Marker.ADD

            msgs.append(msg)

        msgs[0].color.r = 1.0
        msgs[0].pose.orientation.w = 1.0
        msgs[0].scale.x *= inertias[0]/nominal_inertia + 0.2
        msgs[0].id = 1

        msgs[1].color.g = 1.0
        msgs[1].pose.orientation.z = 0.707
        msgs[1].pose.orientation.w = 0.707
        msgs[1].scale.x *= inertias[1]/nominal_inertia + 0.2
        msgs[1].id = 2
        
        msgs[2].color.b = 1.0
        msgs[2].pose.orientation.y = -0.707
        msgs[2].pose.orientation.w = 0.707
        msgs[2].scale.x *= inertias[2]/nominal_inertia + 0.2
        msgs[2].id = 3

        return msgs

    def gen_com_marker(self):
        msg = Marker()
        msg.header.frame_id = "pelvis"
        msg.header.stamp = self.get_clock().now().to_msg()

        com = self._robot.get_com_pos()

        msg.pose.position.x = com[0]
        msg.pose.position.y = com[1]
        msg.pose.position.z = com[2]

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.a = 1.0
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 0.0

        msg.ns = "atlas"
        msg.id = 0
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD

        return msg


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