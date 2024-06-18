from flask import Flask, request
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import numpy as np
import cv2
import base64

model = YOLO("oloy_verdadeiro.pt")
app = Flask(__name__)

@app.route("/infer", methods=["POST"])
def infer():
    data = request.json
    base64_img = data.get("base64_img")
    resultado = inferencer(base64_img)
    return resultado

def inferencer(base64_img):
    try:
        img = base64.b64decode(str(base64_img))
        new_img = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_COLOR)
    except Exception as e:
        return f"Erro na decodificação da imagem base64: {str(e)}"

    results = model.predict(new_img)

    for result in results:
        annotator = Annotator(new_img)
        boxes = result.boxes

        for box in boxes:
            b = box.xyxy[0]
            c = box.cls
            annotator.box_label(b, model.names[int(c)])

    annotated_image = annotator.result()
    _, buffer = cv2.imencode(".jpg", annotated_image)
    inferenced_image = annotated_image.copy()
    base64_inferenced_image = base64.b64encode(buffer).decode("utf-8")
    return base64_inferenced_image

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=3000)


