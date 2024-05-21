import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

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

        self.angular_speed = 0.0
        self.linear_speed = 0.0
        self.is_killed = False

        self.linear_subscriber = self.create_subscription(
            Float32, "linear_speed", self.linear_callback, 10
        )
        self.angular_subscriber = self.create_subscription(
            Float32, "angular_speed", self.angular_callback, 10
        )

        self.kill_service = self.create_service(
            Trigger, "kill_robot_service", self.kill_callback
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Node de serviço de teleop inicializado.")

    def kill_callback(self, request, response):
        """
        Callback para encerrar o serviço de teleop.
        """
        self.get_logger().info("Sinal de Kill recebido")
        self.is_killed = True
        response.success = True
        response.message = "Kill signal received. Robot stopped."
        return response

    def linear_callback(self, msg):
        """
        Callback para receber velocidade linear.
        """
        self.linear_speed = map(msg.data, -100, 100, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.publish_velocities()
        self.get_logger().info(f"Velocidade linear: {self.linear_speed}")

    def angular_callback(self, msg):
        """
        Callback para receber velocidade angular.
        """
        self.angular_speed = map(msg.data, 100, -100, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self.publish_velocities()
        self.get_logger().info(f"Velocidade angular: {self.angular_speed}")

    def spin(self):
        """
        Roda o serviço de teleop.
        """
        try:
            while rclpy.ok() and not self.is_killed:
                rclpy.spin_once(self)
        except Exception as e:
            self.get_logger().error(f"Ocorreu um erro: {e}")
        finally:
            self.stop_robot()
            self.get_logger().info("Serviço de teleop encerrado.")

    def stop_robot(self):
        """
        Para o robô.
        """
        self.current_speed = Twist()
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.publish_velocities()
        self.get_logger().info("Movimento do robô parado.")

    def publish_velocities(self):
        """
        Publica as velocidades no tópico /cmd_vel.
        """
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopService()
    teleop_node.spin()
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
