import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.publisher_ = self.create_publisher(
            CompressedImage, "/camera_feed", qos_profile=qos_profile_sensor_data
        )
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def spin(self):
        try:
            while rclpy.ok():
                time_before = self.get_clock().now()
                ret, frame = self.cap.read()
                if ret:
                    _, buffer = cv2.imencode(".jpg", frame)
                    msg = CompressedImage()
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format = "jpeg"
                    msg.data = buffer.tobytes()
                    self.publisher_.publish(msg)
                    time_after = self.get_clock().now()
                    self.get_logger().info(
                        f"Publishing image at {1/(time_after - time_before).nanoseconds * 1e9} fps"
                    )

                rclpy.spin_once(self, timeout_sec=0.0001)

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

        finally:
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    camera_publisher.spin()
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
