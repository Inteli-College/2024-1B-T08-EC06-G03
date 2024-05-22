#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/obstacle_distances', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.subscription  # apenas para não aparecer o aviso de variável não usada
        self.get_logger().info('LidarProcessor node has been started.')

    def lidar_callback(self, data):
        self.get_logger().info('Dados recebidos pelo tópico /scan.')
        ranges = data.ranges
        distances = [float('inf')] * 8  # [front, front-right, right, back-right, back, back-left, left, front-left]
        
        num_ranges = len(ranges)
        sector_size = num_ranges // 8

        for i in range(8):
            sector_ranges = ranges[i * sector_size:(i + 1) * sector_size]
            min_distance = min(sector_ranges)
            distances[i] = min_distance * 100  # Convertendo para centímetros

        msg = Float32MultiArray()
        msg.data = distances
        self.publisher_.publish(msg)
        
        # Printar as distâncias no terminal
        direction_labels = ['front', 'front-right', 'right', 'back-right', 'back', 'back-left', 'left', 'front-left']
        print("Obstacle distances (in centimeters):")
        for i in range(8):
            print(f"{direction_labels[i]}: {distances[i]:.2f} centimeters")

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
