---
title: Visão Computacional
sidebar_position: 1
---

# Modelo de Visão Computacional - YOLO

Um dos requisitos para o funcionamento desse projeto é a detecção de resíduos dentro dos tubos, a fim de determinar o grau de impureza dentro dos reboilers. Para isso, utilizamos um modelo de [visão computacional](https://aws.amazon.com/pt/what-is/computer-vision/), o [YOLOv8](https://docs.ultralytics.com/models/yolov8/), que é voltado para a detecção de objetos em imagens em tempo real. Os motivos de escolha do modelo, bem como a definição de datasets e implementação da inteligência artificial serão descritos no decorrer dessa seção.

## Escolha do Modelo

O YOLO (You Only Look Once) é um modelo de visão computacional voltado para a detecção de objetos em imagens em tempo real. Proposto por Joseph Redmon e Ali Farhadi em 2015, o YOLO foi escolhido por sua velocidade e precisão na detecção de objetos, sendo capaz de processar imagens a uma taxa de 45 frames por segundo. Além disso, sua capacidade de operar em tempo real é crucial para o contexto do projeto. Comparado a outros modelos de detecção de objetos, o YOLO é menos propenso a predições falsas e pode localizar erros com maior eficiência. No projeto, foi decidida a utilização da versão [YOLOv8](https://docs.ultralytics.com/models/yolov8/), apesar de existirem atualizações mais recentes, porque a instalação é facilitada (não há a necessidade de clonar um repositório, por exemplo, como a [YOLOv9](https://docs.ultralytics.com/models/yolov9/)), e também há mais exemplos encontrados online, facilitando o aprendizado dos desenvolvedores e implementação, já que a documentação é mais rica, o que não é o caso da versão [YOLOv10](https://docs.ultralytics.com/models/yolov10/), que foi lançada durante o desenvolvimento desse projeto, e ainda não tem muitos casos de uso.

## Funcionamento

O YOLO utiliza uma única rede neural convolucional para prever múltiplas caixas delimitadoras e as probabilidades associadas a essas caixas. O modelo divide a imagem em uma grade e, para cada célula da grade, prevê as caixas e suas probabilidades em apenas uma avaliação, tornando-o extremamente eficiente.

## Treinamento do Modelo

### Dataset

O dataset utilizado para treinar o modelo foi o `Precision SG Subterranean`, que contém mais de 260 imagens de ambientes semelhantes a canos de reboilers, focando na detecção de sujeira nesses ambientes. Esse dataset foi escolhido por suas imagens de alta qualidade e por ser de acesso público, o que facilita a replicação dos resultados.

## Resultados

Para uma visualização mais clara dos resultados obtidos com o treinamento do modelo, foi criado um gráfico com parâmetros comumente utilizados para definir a eficácia de um modelo de visão computacional. O gráfico apresenta a precisão, que indica a proporção de predições corretas feitas pelo modelo, o recall, que indica a proporção de objetos detectados corretamente pelo modelo, e o mAP50, que é a média da precisão em diferentes valores de threshold. Estes valores foram calculdados a partir das épocas do treinamento, que indicam o número de vezes que o modelo passou por todo o dataset de treinamento.

<div align="center">

![Resultado](../../static/img/output.png)

**Fonte:** Elaborado pela equipe Rebólins

</div>

O modelo alcançou uma precisão estável entre 80% e 90% em todas as imagens testadas, indicando uma alta capacidade de detecção de sujeira nos canos de reboilers. Além disso, o modelo apresentou um alto recall, demonstrando ser capaz de detectar a maioria dos objetos presentes nas imagens.

## Bibliografia

[1] ALVES, Gabriel. Detecção de Objetos com YOLO – Uma abordagem moderna. Disponível em: [https://iaexpert.academy/2020/10/13/deteccao-de-objetos-com-yolo-uma-abordagem-moderna/?doing_wp_cron=1717618413.5629088878631591796875](https://iaexpert.academy/2020/10/13/deteccao-de-objetos-com-yolo-uma-abordagem-moderna/?doing_wp_cron=1717618413.5629088878631591796875). Acesso em: 06 de junho de 2024.

[2] REDMON, Joseph; DIVVALA, Santosh; GIRSHICK, Ross; FARHADI, Ali. You Only Look Once: Unified, Real-Time Object Detection. Disponível em: [https://arxiv.org/abs/1506.02640](https://arxiv.org/abs/1506.02640). Acesso em: 06 de junho de 2024.