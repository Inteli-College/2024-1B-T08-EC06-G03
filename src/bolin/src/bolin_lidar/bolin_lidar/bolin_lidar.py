import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from math import isnan


class LidarService(Node):
    def __init__(self):
        """
        Inicializa o nó do serviço do Lidar.

        Este nó é responsável por receber dados do sensor Lidar, calcular as distâncias
        dos obstáculos em diferentes direções e publicar essas distâncias em um tópico.

        """
        super().__init__("lidar_data")

        self.publisher_ = self.create_publisher(
            Float32MultiArray, "/obstacle_distances", 10
        )
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, qos_profile=qos_profile_sensor_data
        )

        self.get_logger().info("Nó do Lidar foi iniciado.")

    def lidar_callback(self, data):
        """
        Função de retorno de chamada para o recebimento de dados do sensor Lidar.

        Recebe os dados do sensor Lidar, calcula as distâncias dos obstáculos em diferentes direções,
        publica essas distâncias em um tópico e imprime as distâncias no terminal.

        :param data: Dados recebidos do sensor Lidar.
        """
        distances = self.calculate_distances(data.ranges)
        self.publish_distances(distances)
        self.print_distances(distances)

    def calculate_distances(self, ranges):
        """
        Calcula as distâncias dos obstáculos em diferentes direções.

        :param ranges: Lista de distâncias medidas pelo sensor Lidar.
        :return: Lista de distâncias dos obstáculos em centímetros.
        """
        distances = [float("inf")] * 8
        num_ranges = len(ranges)
        sector_size = num_ranges // 8

        for i in range(8):
            sector_ranges = ranges[i * sector_size : (i + 1) * sector_size]
            sector_ranges = [float("inf") if isnan(x) else x for x in sector_ranges]
            min_distance = min(sector_ranges)
            distances[i] = min_distance * 100

        return distances

    def publish_distances(self, distances):
        """
        Publica as distâncias dos obstáculos em um tópico.

        :param distances: Lista de distâncias dos obstáculos em centímetros.
        """
        msg = Float32MultiArray()
        msg.data = distances
        self.publisher_.publish(msg)

    def print_distances(self, distances):
        """
        Imprime as distâncias dos obstáculos no terminal.

        :param distances: Lista de distâncias dos obstáculos em centímetros.
        """
        direction_labels = [
            "front",
            "front-right",
            "right",
            "back-right",
            "back",
            "back-left",
            "left",
            "front-left",
        ]
        print("Distâncias dos obstáculos (em centímetros):")
        for i in range(8):
            print(f"{direction_labels[i]}: {distances[i]:.2f} centímetros")


def main(args=None):
    """
    Função principal para inicializar o nó do serviço do Lidar.

    :param args: Argumentos da linha de comando (opcional).
    """
    rclpy.init(args=args)
    node = LidarService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
