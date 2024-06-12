import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        
        # Cria um publisher para o tópico 'battery_state'
        self.pub_battery_state = self.create_publisher(BatteryState, 'battery_state', 10)
        
        # Configura um timer para publicar a cada 20 milissegundos
        self.timer = self.create_timer(0.02, self.publish_battery_state)
        
    def publish_battery_state(self):
        # Cria uma mensagem do tipo BatteryState com valores mockados
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.header.frame_id = 'battery_frame'
        battery_msg.voltage = 12.1  # Valor mockado de voltagem
        battery_msg.temperature = 25.0  # Valor mockado de temperatura
        battery_msg.current = 0.0  # Valor mockado de corrente
        battery_msg.charge = 10.0  # Valor mockado de carga
        battery_msg.capacity = 20.0  # Valor mockado de capacidade
        battery_msg.design_capacity = 20.0  # Capacidade de design mockada
        battery_msg.percentage = 0.5  # 50% de carga
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        battery_msg.present = True  # A bateria está presente
        
        # Publica a mensagem no tópico 'battery_state'
        self.pub_battery_state.publish(battery_msg)
        
        # Adiciona logs para acompanhar os valores que estão sendo publicados
        self.get_logger().info(f'Publicado no tópico battery_state:')
        self.get_logger().info(f'  Porcentagem da Bateria: {battery_msg.percentage * 100:.2f}%')
        self.get_logger().info(f'  Voltagem da Bateria: {battery_msg.voltage:.2f}V')

def main(args=None):
    # Inicializa o ROS 2
    rclpy.init(args=args)
    
    # Cria a instância do nó BatteryPublisher
    battery_publisher = BatteryPublisher()
    
    # Mantém o nó rodando
    rclpy.spin(battery_publisher)
    
    # Destrói o nó e desliga o ROS 2 ao final
    battery_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
