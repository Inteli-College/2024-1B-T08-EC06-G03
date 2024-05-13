import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import subprocess


TURTLESIM = False


class TeleopService(Node):
    def __init__(self):
        """
        Inicializa o nó de serviço de teleop.
        """
        super().__init__("teleop_service")
        self.teleop_process = None
        self.kill_subscriber = self.create_subscription(
            String, "kill_button", self.kill_callback, 10
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

    def start_teleop_service(self):
        """
        Inicia o serviço de teleop.
        """
        if TURTLESIM:
            self.teleop_process = subprocess.Popen(
                ["ros2", "run", "turtlesim", "turtlesim_node"]
            )
        else:
            self.teleop_process = subprocess.Popen(
                ["ros2", "launch", "turtlebot3_bringup", "robot.launch.py"]
            )
        self.get_logger().info("Serviço de teleop iniciado.")

    def stop_teleop_service(self):
        """
        Encerra o serviço de teleop.
        """
        if self.teleop_process:
            self.teleop_process.terminate()
            self.teleop_process = None
            self.get_logger().info("Serviço de teleop encerrado.")

    def kill_callback(self, msg):
        """
        Callback para encerrar o serviço de teleop.
        """
        self.stop_teleop_service()

    def linear_callback(self, msg):
        """
        Callback para receber velocidade linear.
        """
        self.linear_speed = msg.data
        self.publish_velocity()
        self.get_logger().info(f"Velocidade linear: {self.linear_speed}")

    def angular_callback(self, msg):
        """
        Callback para receber velocidade angular.
        """
        self.angular_speed = msg.data
        self.publish_velocity()
        self.get_logger().info(f"Velocidade angular: {self.angular_speed}")

    def publish_velocity(self):
        """
        Publica a velocidade no tópico /cmd_vel.
        """
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopService()
    teleop_node.start_teleop_service()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
