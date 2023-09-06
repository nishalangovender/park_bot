import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import yaml


class CommandSender(Node):
    def __init__(self):
        super().__init__('command_sender')
        self.steering_pub = self.create_publisher(
            Float64, 'steering_command_topic', 10)
        self.wheels_pub = self.create_publisher(
            Twist, 'wheels_velocity_command_topic', 10)

        # Read commands from the YAML file
        with open('commands.yaml', 'r') as yaml_file:
            commands = yaml.safe_load(yaml_file)

        # Publish steering commands
        delta_A = commands['steering_command']['delta_A']
        delta_B = commands['steering_command']['delta_B']
        self.steering_pub.publish(Float64(data=delta_A))
        self.steering_pub.publish(Float64(data=delta_B))

        # Publish wheel commands
        front_wheel_velocity = commands['wheel_command']['front_wheel_velocity']
        rear_wheel_velocity = commands['wheel_command']['rear_wheel_velocity']
        self.publish_wheel_commands(front_wheel_velocity, rear_wheel_velocity)

    def publish_wheel_commands(self, front_velocity, rear_velocity):
        twist_msg = Twist()
        twist_msg.linear.x = front_velocity
        twist_msg.linear.y = rear_velocity
        self.wheels_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    command_sender = CommandSender()
    rclpy.spin(command_sender)
    command_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
