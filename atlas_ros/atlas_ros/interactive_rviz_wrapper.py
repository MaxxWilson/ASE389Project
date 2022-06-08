from this import d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import pinocchio as pin

class InteractiveRVIZWrapper(Node):
    def __init___(self) -> None:
        super().__init__("interactive_rviz_wrapper")

        self.joint_state_subscriber = self.create_subscription(JointState, "/joint_state", self.joint_state_callback, 10)

    def joint_state_callback(msg: JointState) -> None:
        print(20 * "-")
        for i in range(msg.name):
            print("Name: " + str(msg.name[i]))
            print("Position: " + str(msg.position[i]))
            print()

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveRVIZWrapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()