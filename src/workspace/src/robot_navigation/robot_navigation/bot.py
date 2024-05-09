import rclpy
from geometry_msgs.msg import Twist

#Função de movimentação do robô
def move_robot(direction):
    rclpy.init()
    node = rclpy.create_node('move_robot_publisher')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    msg = Twist() #tipo de mensagem recebida pelo cmd/vel

    # Define as velocidades lineares e angulares de acordo com a direção
    if direction == 'frente':
        msg.linear.x = 0.2  # velocidade linear para frente
        msg.angular.z = 0.0
    elif direction == 'trás':
        msg.linear.x = -0.2  # velocidade linear para trás
        msg.angular.z = 0.0
    elif direction == 'esquerda':
        msg.linear.x = 0.0
        msg.angular.z = 0.5  # velocidade angular para girar para a esquerda
    elif direction == 'direita':
        msg.linear.x = 0.0
        msg.angular.z = -0.5  # velocidade angular para girar para a direita
    else:
        # Se a direção não for reconhecida, pare o robô (aka movimentação de emergência)
        msg.linear.x = 0.0
        msg.angular.z = 0.0

    pub.publish(msg)

    # Desliga o nó ao finalizar
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Exemplo de uso da função
    move_robot(direction)  # Move o robô para frente




'''
Código de teste para a conexão do robô!!!

import rclpy
from geometry_msgs.msg import Twist

def move_turtle():
    rclpy.init()
    node = rclpy.create_node('move_turtle_publisher')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    msg = Twist()

    # Define a velocidade linear e angular desejada
    msg.linear.x = 0.2  # velocidade linear em m/s
    msg.angular.z = 0.5  # velocidade angular em rad/s

    while rclpy.ok():
        # Publica a mensagem de velocidade
        pub.publish(msg)
        node.get_logger().info('Publicando mensagem de velocidade')
        rclpy.spin_once(node)

    # Desliga o nó ao finalizar
    node.destroy_node()
    rclpy.shutdown()

def main():
    move_turtle()

if __name__ == '__main__':
    main()
'''