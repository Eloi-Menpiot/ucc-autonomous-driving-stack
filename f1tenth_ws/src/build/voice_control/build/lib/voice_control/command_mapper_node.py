import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

class CommandMapperNode(Node):
    def __init__(self):
        super().__init__('command_mapper_node')
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.create_subscription(String, '/voice_commands', self.voice_callback, 10)

    def voice_callback(self, msg):
        command = msg.data.lower()
        drive_msg = AckermannDriveStamped()
        if command == "forward":
            drive_msg.drive.speed = 1.0
        elif command == "backward":
            drive_msg.drive.speed = -1.0
        elif command == "left":
            drive_msg.drive.steering_angle = 0.5
        elif command == "right":
            drive_msg.drive.steering_angle = -0.5
        elif command == "stop":
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommandMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
