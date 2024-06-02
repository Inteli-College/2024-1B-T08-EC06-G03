import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import time


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.subscription = self.create_subscription(
            CompressedImage, "/camera_feed", self.listener_callback, 10
        )
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.total_frames = 0
        self.total_time = 0
        self.avg_fps = 0
        self.avg_time = 0
        self.measurement_count = 0
        self.start_time = time.time()
        self.end_time = self.start_time + 5  # Time interval for calculating average FPS
        self.finish_measurement_time = 0
        self.finish_time = self.start_time + 10 * 5  # Total time for measurements

    def listener_callback(self, msg):
        current_time = time.time()
        elapsed_time = current_time - self.last_frame_time
        self.last_frame_time = current_time
        self.frame_count += 1
        self.total_frames += 1
        self.total_time += elapsed_time

        if elapsed_time > 0:
            fps = 1.0 / elapsed_time
            self.avg_fps = (self.avg_fps * self.measurement_count + fps) / (
                self.measurement_count + 1
            )
            self.avg_time = (self.avg_time * self.measurement_count + elapsed_time) / (
                self.measurement_count + 1
            )

        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display FPS on the image
        cv2.putText(
            image,
            f"FPS: {fps:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow("Camera Feed", image)
        cv2.waitKey(1)

        if current_time >= self.end_time:
            self.finish_measurement_time = time.time()
            self.measurement_count += 1
            if self.measurement_count == 10:
                self.shutdown()

            print(f"Measurement {self.measurement_count}:")
            print(f"Average FPS in last 5 seconds: {self.avg_fps:.2f}")
            self.avg_fps = 0
            self.avg_time = 0
            self.start_time = time.time()
            self.end_time = self.start_time + 5

    def shutdown(self):
        self.finish_time = time.time()
        overall_avg_fps = self.total_frames / self.total_time
        overall_avg_time = self.total_time / self.total_frames
        print("\nProgram finished after 10 measurements:")
        print(f"Overall Average FPS: {overall_avg_fps:.2f}")
        print(f"Overall Average Time per Frame: {overall_avg_time:.6f} seconds")
        print(f"Total Time: {self.finish_time - self.start_time:.2f} seconds")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
