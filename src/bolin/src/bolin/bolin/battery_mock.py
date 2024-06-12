import time
import rclpy
from std_msgs.msg import Header
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryPercentagePublisher(Node):
    def __init__(self):
        super().__init__('battery_percentage_publisher')
        
        # Cria o publisher para o tópico 'battery_state'
        self.publisher = self.create_publisher(BatteryState, 'battery_state', 10)
        
        # Cria um timer para chamar a função de envio de mensagens a cada 1 segundo
        timer_period = 1.0  # segundos
        self.timer = self.create_timer(timer_period, self.send_battery_percentage)
        
    def send_battery_percentage(self):
        # Cria uma mensagem do tipo BatteryState com valores mockados
        battery_msg = BatteryState(
            header=self.get_msg_header(),
            voltage=12.09,
            temperature=25.0,
            current=0.0,
            charge=10.0,
            capacity=20.0,
            design_capacity=20.0,
            percentage=50.0,
            power_supply_status=BatteryState.POWER_SUPPLY_STATUS_UNKNOWN,
            power_supply_health=BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN,
            power_supply_technology=BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
            present=True,
            cell_voltage=[],
            cell_temperature=[],
            location='MockLocation',
            serial_number='123456789'
        )
        
        # Publica a mensagem no tópico
        self.publisher.publish(battery_msg)
        self.get_logger().info(f'Published battery percentage: {battery_msg.percentage}%')

    def get_msg_header(self):
        # Cria um header para a mensagem com o timestamp atual
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'battery_frame'
        return header

def main(args=None):
    # Inicializa o ROS 2
    rclpy.init(args=args)
    
    # Cria o nó de publicação
    battery_publisher_node = BatteryPercentagePublisher()
    
    # Faz o ROS 2 manter o nó rodando
    rclpy.spin(battery_publisher_node)
    
    # Desliga o ROS 2 quando o processo terminar
    rclpy.shutdown()

if __name__ == '__main__':
    main()
