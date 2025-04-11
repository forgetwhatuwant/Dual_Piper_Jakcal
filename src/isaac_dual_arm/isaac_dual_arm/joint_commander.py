import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointCommander(Node):

    def __init__(self):
        super().__init__('joint_commander')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # List of joint names to command (update with your actual joint names)
        self.joint_names = ['left_joint1', 'right_joint1'] # Example joints

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # --- Specify the joints to command ---
        msg.name = self.joint_names

        # --- Specify the target positions (must match order in msg.name) ---
        # Example: Move left_joint1 to 0.5 rad and right_joint1 to -0.5 rad
        target_positions = [0.5, -0.5]
        msg.position = [float(p) for p in target_positions]

        # --- Optionally specify velocities or efforts if needed ---
        # msg.velocity = [0.1, 0.1] # Example velocities
        # msg.effort = [] # Example efforts (often left empty for position control)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing JointState: names={msg.name}, positions={msg.position}')
        # self.i += 1 # Example: if you want to change command over time


def main(args=None):
    rclpy.init(args=args)
    joint_commander = JointCommander()

    try:
        rclpy.spin(joint_commander)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        joint_commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 