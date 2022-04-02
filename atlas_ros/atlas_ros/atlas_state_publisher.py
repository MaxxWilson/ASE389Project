
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class AtlasStatePublisher():
    def __init__(self):
        rclpy.init()
        self.node = Node("atlas_state_publisher")
        self.joint_state_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        self.transform_pub = self.node.create_publisher(TFMessage, "/tf", 10)
        
    def publish_robot_state(self, robot_state):
        joint_msg = JointState()
        
        # Set msg time stamp using ros time
        joint_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        # Publish position and velocity of robot joints
        joint_pos = robot_state.get("joint_pos")
        joint_vel = robot_state.get("joint_vel")
        
        for key in joint_pos:
            joint_msg.name.append(key)
            joint_msg.position.append(joint_pos.get(key))
            joint_msg.velocity.append(joint_vel.get(key))
            
        self.joint_state_pub.publish(joint_msg)
        
        # Publish world to com transform
        tf_stamped_msg = TransformStamped()
        
        # Set msg time stamp using ros time
        tf_stamped_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        base_joint_pos = robot_state.get("base_joint_pos")
        base_joint_quat = robot_state.get("base_joint_quat")
        
        tf_stamped_msg.header.frame_id = "world"
        tf_stamped_msg.child_frame_id = "pelvis"
        tf_stamped_msg.transform.translation.x = base_joint_pos[0]
        tf_stamped_msg.transform.translation.y = base_joint_pos[1]
        tf_stamped_msg.transform.translation.z = base_joint_pos[2]
        tf_stamped_msg.transform.rotation.x = base_joint_quat[0]
        tf_stamped_msg.transform.rotation.y = base_joint_quat[1]
        tf_stamped_msg.transform.rotation.z = base_joint_quat[2]
        tf_stamped_msg.transform.rotation.w = base_joint_quat[3]
        
        # Cube Publish?
        # Publish world to com transform
        cube_stamped_msg = TransformStamped()
        
        # Set msg time stamp using ros time
        cube_stamped_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        cube_stamped_msg.header.frame_id = "world"
        cube_stamped_msg.child_frame_id = "cube_center"
        cube_stamped_msg.transform.translation.x = 2.0
        cube_stamped_msg.transform.translation.y = 0.0
        cube_stamped_msg.transform.translation.z = 0.5
        
        # Publish all TF Frames
        tf_msg = TFMessage()
        tf_msg.transforms.append(cube_stamped_msg)
        tf_msg.transforms.append(tf_stamped_msg)
        
        self.transform_pub.publish(tf_msg)
        
    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        