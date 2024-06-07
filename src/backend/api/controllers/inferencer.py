import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator

model = YOLO('best.pt')

class YOLOv8Inference(Node):
    def __init__(self, image_path):
        super().__init__('yolov8_inference')
        self.image_path = image_path
        self.publisher_ = self.create_publisher(CompressedImage, '/inferenced_camera_feed', 10)
        self.cv_bridge = CvBridge()

    def process_image(self):
        try:
            cv_image = cv2.imread(self.image_path)

            results = model.predict(cv_image)

            for r in results:
                annotator = Annotator(cv_image)
                boxes = r.boxes

                for box in boxes:
                    b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                    c = box.cls
                    annotator.box_label(b, model.names[int(c)])
            
            cv_img = annotator.result()

            inferenced_image = cv_img.copy()

            # Convert inferenced_image back to CompressedImage
            inferenced_msg = self.cv_bridge.cv2_to_compressed_imgmsg(inferenced_image)
            self.publisher_.publish(inferenced_msg)
        
        except Exception as e:
            print(e)

def main(args=None):
    parser = argparse.ArgumentParser(description='Process an image with YOLOv8.')
    parser.add_argument('image_path', help='The path to the image to be processed.')
    args = parser.parse_args()

    rclpy.init(args=args)
    yolov8_inference_node = YOLOv8Inference(args.image_path)
    yolov8_inference_node.process_image()
    yolov8_inference_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()