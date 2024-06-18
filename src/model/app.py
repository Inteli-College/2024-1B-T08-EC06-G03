from flask import Flask, request
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import numpy as np
import cv2
import base64

model = YOLO("oloy.pt")
app = Flask(__name__)

@app.route("/infer", methods=["POST"])
def infer():
    data = request.json
    base64_img = data.get("base64_img")
    resultado = inferencer(base64_img)
    #resultado= decode_string(base64_img)
    return resultado

def decode_string(string):
    new_string = base64.b64decode(str(string))
    return new_string

def inferencer(base64_img):
    try:
        img = base64.b64decode(str(base64_img))
    except Exception as e:
        return f"Erro na decodificação da imagem base64: {str(e)}"

    results = model.predict(base64_image=img)
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

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=3000)


