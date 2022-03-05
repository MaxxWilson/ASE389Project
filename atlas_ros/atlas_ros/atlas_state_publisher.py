
import rclpy
from rclpy.node import Node

from collections import OrderedDict
from sensor_msgs.msg import JointState

class AtlasStatePublisher():
    def __init__(self):
        rclpy.init()
        self.node = Node("atlas_state_publisher")
        self.js_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        
    def publish_robot_state(self, robot_state):
        newMsg = JointState()
        newMsg.header.stamp = self.node.get_clock().now().to_msg()
        joint_pos = robot_state.get("joint_pos")
        joint_vel = robot_state.get("joint_vel")
        
        for key in joint_pos:
            newMsg.name.append(key)
            newMsg.position.append(joint_pos.get(key))
            newMsg.velocity.append(joint_vel.get(key))
            
        self.js_pub.publish(newMsg)
            
    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        