import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 40)
        self.timer = self.create_timer(0.01, self.timer_callback)  # Publish every 0.1 seconds (10 Hz)
        self.cap = cv2.VideoCapture(2)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()