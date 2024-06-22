from flask import Flask, request
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import numpy as np
import cv2
import base64

model = YOLO("best_100e.pt")
app = Flask(__name__)

# Route to infer the image sent by the client
# Returns a json with the base64 inferenced image and a flag indicating if dirt was detected
@app.route("/infer", methods=["POST"])
def infer():
    data = request.json
    base64_img = data.get("base64_img")
    base64_inferenced_image, dirt_detected = inferencer(base64_img)
    return {"base64_infered_img": base64_inferenced_image,
            "dirt_detected": dirt_detected}

# Function to infer the image sent 
# Returns the inferenced image plus a flag indicating if dirt was detected
def inferencer(base64_img):
    dirt_detected = 0
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
            if model.names[int(c)] == "dirt":
                dirt_detected = 1

    annotated_image = annotator.result()
    _, buffer = cv2.imencode(".jpg", annotated_image)
    base64_inferenced_image = base64.b64encode(buffer).decode("utf-8")
    return base64_inferenced_image, dirt_detected

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=3000)


