---
title: Backend Python
sidebar_position: 3
---

Embora o backend do projeto tenha sido majoritariamente desenvolvido em Javascript, foi implementada uma parte em Python. Foi assim para que o modelo de visão computacional, executado em Python, pudesse ser melhor integrado à solução.

## Da arquitetura
Esta seção não planeja explicar a arquitetura geral do projeto, mas apenas a parte da arquitetura sobre as interações das quais participa o backend python.

<div align="center">

**Arquitetura**

![Arquitetura](/img/backend_python_architecture.jpg)

**Fonte:** Elaborado pela equipe Rebólins

</div>

O backend python permite realizar inferência em uma imagem com o modelo de visão computacional. A requisição disso começa no frontend, em que o usuário aperta um botão na interface após decidir fazer a inferência na imagem visualizada.

Essa informação chega a um dos controllers do backend javascript. Esse controller envia a informação da requisição para um service. O service, por sua vez, faz o fetch para a rota do backend python. A requisição é do tipo POST, e em seu corpo é passado um JSON com a imagem codificada em base64.

O backend python recebe o base64 da imagem, decodifica e utiliza o modelo treinado para inferir em cima dela. Ele retorna, então, um JSON contendo o base64 da imagem com os bounding boxes em torno do objeto detectado, além de uma flag sobre se houve objetos detectados ou não.

O service recebe a resposta e envia de volta para o backend javascript. Quando a resposta chega no backend javascript, ela é devidamente inserida no banco de dados.

Por fim, a imagem com as bounding boxes retorna ao frontend para o usuário visualizá-la.

> :brain:
> **A decisão de implementar um service foi para que a lógica do backend python estivesse o mais apartada possível da lógica do backend javascript.**

## Da implementação do código
Foram importadas algumas bibliotecas. Vale mencionar o Flask, microframework utilizado para criar um endpoint específico; YOLO, da ultralytics, utilizado para carregar o modelo e realizar a predição; Annotator, também da ultralytics, utilizado para escrever as bounding boxes na imagem; cv2, utilizado para lidar com a imagem, assegurando que o modelo reconheça-a adequadamente e base64, utilizado para codificar e decodificar a imagem, facilitando sua trasmissão.

```python
from flask import Flask, request
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import numpy as np
import cv2
import base64
```

---

Definiu-se a rota "/infer", que aceita requisições do tipo POST. O corpo da requisição tem que ser um JSON com um parâmetro chamado "base64_img". O valor desse parâmetro é passado como um argumento para a função "inferencer()" descrita abaixo e o retorno disso é passado na resposta da função do endpoint, que também é em formato JSON.

```python
@app.route("/infer", methods=["POST"])
def infer():
    data = request.json
    base64_img = data.get("base64_img")
    base64_inferenced_image, dirt_detected = inferencer(base64_img)
    return {"base64_infered_img": base64_inferenced_image,
            "dirt_detected": dirt_detected}
```

---

A função "inferencer()" foi definida com um único parâmetro, o base64 da imagem. Uma variável booleana "dirt_detected" foi definida no início para armazenar a informação de sujeira detectada. A função prossegue decodificando a imagem e utilizando o método "predict()" do modelo para fazer a inferência. O loop que segue serve para escrever as bounding boxes na imagem, além de mudar o valor de "dirt_detected" para 1 caso algum objeto da classe "dirt" seja identificado.

```python
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
```

## Rotas
Para ver testes da rota implementada acima, acesse o [Postman](https://www.postman.com/planetary-resonance-766487/workspace/backend-python/request/27247540-477f79a6-5265-430d-9a24-94858efd368a).