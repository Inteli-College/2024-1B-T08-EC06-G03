import rclpy
from geometry_msgs.msg import Twist
from flask import Flask,request, jsonify


app = Flask(__name__)

rclpy.init()
node = rclpy.create_node('move_robot_publisher')
pub = node.create_publisher(Twist, '/cmd_vel', 10)


#Função de movimentação do robô
def move_robot(direction):
    msg = Twist() #tipo de mensagem recebida pelo cmd/vel

    # Define as velocidades lineares e angulares de acordo com a direção
    if direction == 'frente':
        msg.linear.x = 0.1  # velocidade linear para frente
        msg.angular.z = 0.0
    elif direction == 'trás':
        msg.linear.x = -0.1  # velocidade linear para trás
        msg.angular.z = 0.0
    elif direction == 'esquerda':
        msg.linear.x = 0.0
        msg.angular.z = 0.05  # velocidade angular para girar para a esquerda
    elif direction == 'direita':
        msg.linear.x = 0.0
        msg.angular.z = -0.05  # velocidade angular para girar para a direita
    else:
        # Se a direção não for reconhecida, pare o robô (aka movimentação de emergência)
        msg.linear.x = 0.0
        msg.angular.z = 0.0

    print(msg)

    pub.publish(msg)



@app.route('/move', methods=['POST'])
def move():
    if not request.is_json:
        return jsonify({"error": "Missing JSON in request"}), 400
    data = request.get_json()
    direction = data.get('direction')
    if not direction:
        return jsonify({"error": "Missing 'direction' in JSON"}), 400
    move_robot(direction)
    return jsonify({"status": "Movement command executed"}), 200

def main():
    app.run()
    # Desliga o nó ao finalizar
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

