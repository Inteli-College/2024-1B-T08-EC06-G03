import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.pub_level = self.create_publisher(Float32, 'battery_level', 10)
        self.pub_voltage = self.create_publisher(Float32, 'battery_voltage', 10)
        self.subscription = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            10
        )
        self.subscription  # Evitar advertência de variável não utilizada

    def battery_callback(self, msg):
        battery_level = Float32()
        battery_level.data = msg.percentage
        self.pub_level.publish(battery_level)

        battery_voltage = Float32()
        battery_voltage.data = msg.voltage
        self.pub_voltage.publish(battery_voltage)

        # Adicionando logs para imprimir os valores da bateria no terminal
        self.get_logger().info(f'Bateria: {msg.percentage * 100:.2f}%')
        self.get_logger().info(f'Voltagem da Bateria: {msg.voltage:.2f}V')

def main(args=None):
    rclpy.init(args=args)
    battery_publisher = BatteryPublisher()
    rclpy.spin(battery_publisher)
    battery_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
