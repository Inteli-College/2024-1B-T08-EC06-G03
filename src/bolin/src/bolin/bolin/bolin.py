import rclpy
from rclpy.node import Node
from std_msgs.msg import  Float32, Bool
from geometry_msgs.msg import Twist
import subprocess

MAX_LINEAR_VEL = 0.22
MAX_ANGULAR_VEL = 2.84


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class TeleopService(Node):
    def __init__(self):
        """
        Inicializa o nó de serviço de teleop.
        """
        super().__init__("teleop_service")
        self.teleop_process = None
        self.killed = False
        self.kill_subscriber = self.create_subscription(
            Bool, "kill_button", self.kill_callback, 10
        )
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.linear_subscriber = self.create_subscription(
            Float32, "linear_speed", self.linear_callback, 10
        )
        self.angular_subscriber = self.create_subscription(
            Float32, "angular_speed", self.angular_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Node de serviço de teleop inicializado.")

    def kill_callback(self, msg):
        """
        Callback para encerrar o serviço de teleop.
        """
        self.killed = msg.data
        self.get_logger().info(f"Killed recebeu: {self.killed}")

    def linear_callback(self, msg):
        """
        Callback para receber velocidade linear.
        """
        self.linear_speed = map(msg.data, -100, 100, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.publish_velocity()
        self.get_logger().info(f"Velocidade linear: {self.linear_speed}")

    def angular_callback(self, msg):
        """
        Callback para receber velocidade angular.
        """
        self.angular_speed = map(msg.data, -100, 100, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self.publish_velocity()
        self.get_logger().info(f"Velocidade angular: {self.angular_speed}")

    def publish_velocity(self):
        """
        Publica a velocidade no tópico /cmd_vel.
        """
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        
        if self.killed:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist)
            


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopService()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
