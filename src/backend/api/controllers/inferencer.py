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
    def __init__(self):
        super().__init__('yolov8_inference')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_feed',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(CompressedImage, '/inferenced_camera_feed', 10)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            #print(f'np_arr: {np_arr}')
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #print(f'cv_image: {cv_image}')

            results = model.predict(cv_image)
            #print(f'results: {results}')

            for r in results:
                annotator = Annotator(cv_image)
                boxes = r.boxes

                for box in boxes:
                    b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                    c = box.cls
                    annotator.box_label(b, model.names[int(c)])
            
            cv_img = annotator.result()

            # Perform YOLOv8 inference on cv_image
            # Replace the following lines with your YOLOv8 inference code
            # Assuming here that inferenced_image is the result of your inference
            inferenced_image = cv_img.copy()
            # Draw bounding boxes on inferenced_image (sample code)
            # Assuming bboxes contains bounding box coordinates [(x1, y1, x2, y2), ...]
            #for bbox in bboxes:
                #cv2.rectangle(inferenced_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            
            # Convert inferenced_image back to CompressedImage
            inferenced_msg = self.cv_bridge.cv2_to_compressed_imgmsg(inferenced_image)
            inferenced_msg.header = msg.header
            self.publisher_.publish(inferenced_msg)
        
        except Exception as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    yolov8_inference_node = YOLOv8Inference()
    rclpy.spin(yolov8_inference_node)
    yolov8_inference_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

