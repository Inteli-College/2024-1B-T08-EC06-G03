from flask import Flask, request
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import cv2
import base64

app = Flask(__name__)

@app.route("/infer", method=["POST"])
def infer():
    data = request.json
    base64_img = data.get("base64_img")
    return inferencer(base64_img)

model = YOLO("oloy.pt")

def inferencer(base64_img):
    img = base64.b64decode(str(base64_img))
    results = model.predict(img)
    for result in results:
        annotator = Annotator(img)
        boxes = result.boxes

        for box in boxes:
            b = box.xyxy[0]
            c = box.cls
            annotator.box_label(b, model.names[int(c)])

    annotated_image = annotator.result()
    inferenced_image = annotated_image.copy()
    base64_inferenced_img = base64.b64encode(inferenced_image)
    return base64_inferenced_img




