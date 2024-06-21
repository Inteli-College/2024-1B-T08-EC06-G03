---
title: Visão Computacional
sidebar_position: 3
---

# Modelo de Visão Computacional - YOLO

Um dos requisitos para o funcionamento desse projeto é a detecção de resíduos dentro dos tubos, a fim de determinar o grau de impureza dentro dos reboilers. Para isso, utilizamos um modelo de [visão computacional](https://aws.amazon.com/pt/what-is/computer-vision/), o [YOLOv8](https://docs.ultralytics.com/models/yolov8/), que é voltado para a detecção de objetos em imagens em tempo real.

## Escolha de versão do modelo

No projeto, foi decidida a utilização da versão [YOLOv8](https://docs.ultralytics.com/models/yolov8/), apesar de existirem atualizações mais recentes, porque a instalação é facilitada (não há a necessidade de clonar um repbositório, por exemplo, como a [YOLOv9](https://docs.ultralytics.com/models/yolov9/)), e também há mais exemplos encontrados online, facilitando o aprendizado dos desenvolvedores e implementação, já que a documentação é mais rica, o que não é o caso da versão [YOLOv10](https://docs.ultralytics.com/models/yolov10/), que foi lançada durante o desenvolvimento desse projeto, e ainda não tem muitos casos de uso.

## Treinamento do Modelo

### Dataset

Para definir o dataset, foi necessário uma série de pesquisas a fim de encontrar um que fosse o mais fiel possível e ao mesmo tempo *open source* para ser utilizado no projeto, uma vez que a empresa parceira não pôde disponibilizar imagens dos canos dos reboilers.

O dataset utilizado para treinar o modelo foi o [`Tacomare Computer Vision Project`](https://universe.roboflow.com/project-y7tgq/tacomare), que contém mais de 500 imagens de canos com diferentes graus de sujidade, com mais de 350 delas separadas para treino e 100 separadas para teste, sendo que este identifica as classes "tube" e "dirt" (tubo e sujeira, respectivamente). Esse dataset foi escolhido por possuir imagens de alta qualidade e por ser de acesso público, o que facilita a replicação dos resultados, além de facilmente aplicável para testes e utilização em razão do contexto do projeto, isso com o auxílio da plataforma [`Roboflow`](https://roboflow.com/), que é especializada na disponibilização de datasets para projetos específicos.

<div align="center">

**Exemplos de imagens presentes do dataset utilizado**

![Exemplos de imagens do dataset](../../static/img/val_batch1_pred.jpg)


**Fonte:** Elaborado pelo modelo YOLOv8

</div>

### Implementação

Para a implementação do treinamento do modelo, foram utilizadas as bibliotecas `Roboflow` e `Ultralytics`, que são amplamente utilizadas para treinamento de modelos de visão computacional. A biblioteca `Ultralytics` é uma API que facilita o treinamento de modelos YOLO, enquanto a `Roboflow` é uma plataforma que permite a organização e pré-processamento de imagens para treinamento de modelos de visão computacional. </br>

Além disso, o treinamento foi feito utilizando um caderno Jupyter, ferramenta muito utilizada para este propósito por sua facilidade de uso e interatividade, pelo fato de ser possível visualizar os resultados de cada etapa do treinamento e ajustar os hiperparâmetros do modelo em tempo real.

Abaixo há um exemplo do código utilizado para o treinamento do modelo:

```python
# Instalação das bibliotecas necessárias
from roboflow import Roboflow
from ultralytics import YOLO

# Definição do modelo YOLO
model = YOLO("yolov8n.yaml")

# Treinamento do modelo
results = model.train(data="Tacomare-2/data.yaml", epochs=100) 
```
Por fim, foram utilizadas 100 épocas para o treinamento do modelo. Isto indica que o modelo passou por todo o dataset de treinamento 100 vezes, para garantir que ele aprendesse bem as características das imagens e pudesse fazer previsões precisas. Inicialmente foi feito um treinamento com 5 épocas para avaliar seu desempenho, mais detalhes sobre os resultados obtidos serão apresentados na seção de avaliação do modelo.

## Avaliação do modelo

### Definição das principais métricas utilizadas

As métricas são indicadores que mostram como o modelo está performando, permitindo sua avaliação e como melhorá-lo. As principais métricas utilizadas são a **precisão, recall, precisão e recall, f1 confiança e mean average precision** (mAP - que tem tanto o **mAP50** quanto o **mAP50-95**). Agora, será apresentado o que cada uma delas representa:

- **Precisão:** Apresenta a porcentagem de verdadeiros positivos (ou seja, a porcentagem predições corretas). No contexto do nosso projeto, significa em quantos canos o modelo corretamente identificou sujeiras, o que significa que ter essa métrica alta é interessante para o desempenho do projeto.

- **Recall:** é uma métrica que avalia quantos objetos que deveriam ser detectados o modelo corretamente detectou. Por exemplo, se haviam 10 canos com impureza e o modelo detectou apenas 8, ele tem um recall de 80%. Isso ajuda a compreender a capacidade do modelo de encontrar a sujeira nos canos.

- **mAP50:** É uma métrica que utiliza de um **threshold** de 50%, que nada mais é que um valor que determina a precisão que uma detecção precisa performar para ser considerada correta.
Por exemplo, com um threshold de 50% significa que, para uma detecção ser considerada correta, a sobreposição entre a caixa predita pelo modelo e a caixa real do objeto deve ser de pelo menos 50%. É uma métrica mais simples para avaliar o desempenho do modelo.

- **mAP50-95:** Possui a mesma lógica do anterior, mas possui um threshold variável entre 50% a 95%, com intervalos de 5%, dando uma visão mais completa do modelo, pois avalia quão bem ele se sai em diferentes níveis de sobreposição, sendo um pouco mais rigoroso que o mAP50.

### Outras métricas utilizadas

- **Box Loss:** Mede a precisão da localização das bounding boxes previstas em comparação com as bounding boxes reais dos objetos. Quanto menor o valor, melhor o modelo está performando.
- **Classification Loss:** Avalia a precisão da previsão das classes dos objetos dentro das bounding boxes. A perda de classificação mede o erro entre as classes previstas pelo modelo e as classes reais dos objetos.
- **Ditribution Focal Loss:** Perda usada em tarefas de detecção de objetos, especialmente em arquiteturas modernas como o próprio YOLO. A DFL é uma combinação de aspectos da Focal Loss e técnicas de distribuição para melhorar a precisão da previsão de bounding boxes e classes. Ela se concentra em penalizar previsões incorretas de forma mais severa e, ao mesmo tempo, ajuda a lidar com a distribuição de dados desbalanceada.

### Resultados

Para uma visualização mais clara dos resultados obtidos com o treinamento do modelo, é possível observar no gráfico abaixo o desempenho dele nas métricas **precisão, recall, mAP50 e mAP50-95** durante cada uma das 100 épocas de treinamento. </br>
Além das principais métricas, também é possível observar o desempenho do modelo em relação às métricas **Box Loss, Classification Loss e Distribution Focal Loss**, sendo que as três foram medidas em relação ao treino e validação do modelo.

<div align="center">

**Níveis das métricas em 100 épocas**

![Resultado](../../static/img/results.png)

**Fonte:** Elaborado pelo modelo YOLOv8

</div>

Analisando o gráfico apresentado é possível notar as seguintes informações sobre as métricas avaliadas:

1. **train/box_loss**: Mostra a perda da bounding box durante o treinamento. Observa-se uma queda significativa de cerca de 3,5 para aproximadamente 0,5, indicando que o modelo está aprendendo a localizar as bounding boxes de maneira mais precisa conforme o treinamento avança.

2. **train/cls_loss**: Representa a perda de classificação durante o treinamento. Esta perda também diminui de aproximadamente 3,5 para cerca de 0,5, sugerindo que o modelo está melhorando na classificação dos objetos dentro das bounding boxes ao longo do treinamento.

3. **train/dfl_loss**: Refere-se à perda de focalização de distribuição (Distribution Focal Loss) durante o treinamento. Essa perda diminui de cerca de 4 para 1, indicando melhorias na previsão da distribuição das bounding boxes.

4. **metrics/precision(B)**: Mostra a precisão do modelo durante o treinamento. A precisão inicial é baixa, mas rapidamente sobe para cerca de 1, indicando que o modelo se torna altamente preciso na detecção de objetos após algumas épocas.

5. **metrics/recall(B)**: Reflete o recall do modelo durante o treinamento. Inicialmente, o recall é baixo, mas rapidamente aumenta para 1, mostrando que o modelo está se tornando eficaz em detectar a maioria dos objetos presentes nas imagens.

6. **val/box_loss**: Mostra a perda da bounding box durante a validação. Similar ao gráfico de treinamento, a perda cai de cerca de 3,5 para aproximadamente 0,5, indicando que o modelo está se generalizando bem em dados de validação.

7. **val/cls_loss**: Representa a perda de classificação durante a validação. Essa perda diminui de aproximadamente 3,5 para cerca de 0,5, sugerindo melhorias na classificação de objetos em dados de validação.

8. **val/dfl_loss**: Refere-se à perda de focalização de distribuição durante a validação. Essa perda diminui de cerca de 4 para 1, similar ao comportamento observado durante o treinamento.

9. **metrics/mAP50(B)**: Mostra a média de precisão (mean Average Precision) com um limiar de 50% durante a validação. A mAP50 rapidamente atinge cerca de 1, indicando alta precisão na detecção de objetos com esse limiar.

10. **metrics/mAP50-95(B)**: Reflete a mAP (mean Average Precision) com múltiplos limiares (50-95%) durante a validação. A mAP50-95 aumenta gradualmente, alcançando cerca de 0,85, sugerindo que o modelo está se tornando progressivamente melhor em detectar objetos com uma variedade de limiares.

Por fim, é possível observar que o modelo YOLOv8 obteve um desempenho muito bom em todas as métricas avaliadas, indicando que ele é capaz de detectar objetos com alta precisão e recall, além de ter uma alta mAP50 e mAP50-95, o que sugere que ele é capaz de detectar objetos com diferentes níveis de confiança. Em relação às perdas de bounding box, classificação e focalização de distribuição, todas diminuíram significativamente ao longo do treinamento, o que indica que o modelo está aprendendo a localizar as bounding boxes, classificar os objetos e prever sua distribuição de maneira mais precisa.

### Comparação entre os modelos treinados

Antes de finalizar o treinamento do modelo, foi feito um treinamento com 5 épocas para avaliar seu desempenho inicial. Abaixo, é possível observar um gráfico com algumas das métricas utilizadas para medir o modelo de 100 épocas, porém com 5:

<div align="center">

**Métricas após treinamento de 5 épocas**

![Resultado](../../static/img/results_5e.png)

**Fonte:** Elaborado pelo modelo YOLOv8

</div>

Analisando o gráfico acima, é notável que, apesar de apresentar bons resultados, o modelo mostrava tendência a melhorar em todas as métricas, mas ainda não havia atingido o seu potencial máximo. Por exemplo, a precisão mostra um aumento repentino na última época, enquanto o recall apresenta uma leve queda, o que pode sugerir um equilíbrio entre precisão e recall que ainda precisava ser ajustado. Além disso, a melhoria nas métricas mAP50 e mAP50-95 sugere que o modelo poderia se tornar mais eficaz em detectar objetos com diferentes níveis de confiança.

## Testes e conclusão

O modelo YOLOv8 foi treinado com sucesso para detectar sujeira em canos de reboilers, atingindo altas pontuações em métricas seja em precisão, recall, mAP50 e mAP50-95 ao ser treinado com 100 épocas, sendo capaz de detectar todos os objetos presentes nas imagens com alta precisão.

Abaixo é possível observar um exemplo de uma imagem de teste com a detecção de sujeira realizada pelo modelo:

<div align="center">

**Primeiro exemplo de detecção de sujeira em canos**

![Detecção de sujeira](../../static/img/val_batch0_pred.jpg)

**Fonte:** Elaborado pelo modelo YOLOv8


</div>

Observando esta imagem, pode-se observar que o modelo corroborou com as métricas apresentadas, detectando tubos e sujeiras com uma eficácia muito alta e sem erros aparentes.

<div align="center">

**Segundo exemplo de detecção de sujeira em canos**

![Detecção de sujeira](../../static/img/train_batch2.jpg)

**Fonte:** Elaborado pelo modelo YOLOv8

</div>

Nesta segunda imagem, é possível notar que o modelo foi capaz de detectar tanto os tubos (representados por 0) quanto a sujeira (representado por 1) presente neles em diferentes posições e ângulos, isto mostra a versatilidade do modelo em realizar detecções em cenários distintos e específicos.

Por fim, é importante ressaltar que o modelo foi treinado com um dataset específico e, portanto, é importante avaliar o desempenho do modelo em um ambiente de produção para garantir que ele seja capaz de generalizar bem para novos dados. Além disso, é importante monitorar o desempenho do modelo ao longo do tempo.

## Estrutura de pastas

Para a organização do projeto, foi criada uma estrutura de pastas que facilita a organização e o acesso aos arquivos necessários para o treinamento e avaliação do modelo. Abaixo, é possível observar a estrutura de pastas utilizada:

```
model
│
├── best_5e.pt
├── best_100e.pt
├── oloy_train.ipynb
├── runs
│   ├── detect
│   │   ├── train_5
│   │   │   ├── args.yaml
│   │   │   ├── confusion_matrix_normalized.png
│   │   │   ├── confusion_matrix.png
│   │   │   ├── f1_curve.png
│   │   │   ├── labels_correlogram.png
│   │   │   ├── labels.png
│   │   │   ├── P_curve.jpg
│   │   │   ├── PR_curve.jpg
│   │   │   ├── R_curve.png
│   │   │   ├── results.csv
│   │   │   ├── results.png
│   │   │   ├── test_batch0_pred.jpg
│   │   │   ├── test_batch1_pred.jpg
│   │   │   ├── test_batch2_pred.jpg
│   │   │   ├── val_batch0_labels.jpg
│   │   │   ├── val_batch0_pred.jpg
│   │   │   ├── val_batch1_labels.jpg
│   │   │   ├── val_batch1_pred.jpg
│   │   │   ├── val_batch2_labels.jpg
│   │   │   ├── val_batch2_pred.jpg
│   │   │   ├── weights
│   │   │   │   ├── best.pt
│   │   │   │   ├── last.pt
│   │   ├── train_100
│   │   │   ├── args.yaml
│   │   │   ├── confusion_matrix_normalized.png
│   │   │   ├── confusion_matrix.png
│   │   │   ├── f1_curve.png
│   │   │   ├── labels_correlogram.png
│   │   │   ├── labels.png
│   │   │   ├── P_curve.jpg
│   │   │   ├── PR_curve.jpg
│   │   │   ├── R_curve.png
│   │   │   ├── results.csv
│   │   │   ├── results.png
│   │   │   ├── test_batch0_pred.jpg
│   │   │   ├── test_batch1_pred.jpg
│   │   │   ├── test_batch2_pred.jpg
│   │   │   ├── val_batch0_labels.jpg
│   │   │   ├── val_batch0_pred.jpg
│   │   │   ├── val_batch1_labels.jpg
│   │   │   ├── val_batch1_pred.jpg
│   │   │   ├── val_batch2_labels.jpg
│   │   │   ├── val_batch2_pred.jpg
│   │   │   ├── weights
│   │   │   │   ├── best.pt
│   │   │   │   ├── last.pt
```

A seguir, é possível observar a descrição de cada pasta e arquivo presente na estrutura acima:

**model** -> O arquivo best_5e.pt e o best_100e.pt representa o modelo treinado com 5 épocas e o modelo treinado com 100 épocas, respectivamente, de melhor resultado, sendo que este também está disponível no caminho run/detect/train19/weights. Já o arquivo oloy_train.ipynb representa o caderno que treina o modelo.

**args.yaml** -> Código em formato YAML que define várias configurações para o processo de treinamento de detecção de objetos usando o modelo YOLOv8. O arquivo args.yaml contém uma série de pares chave-valor que especificam diferentes parâmetros e opções para o treinamento.

**runs/detect/train_5** e **runs/detect/train_100** -> Resultados do treinamento mais recente, contém gráficos de análise das métricas como matriz de confusão e gráficos de linha. Além disso, há um arquivo .csv com os resultados dos treinamentos das épocas e imagens de treino que detectam funções disponíveis no dataset, como a de identificar um tubo e de identificar sujeira no mesmo.

**run/detect/train_5/weights** e **run/detect/train_100/weights** -> Possui os dois arquivos do modelo final treinado, um denominado "best.pt" que representa o modelo com os melhores resultados e o "last.pt", que representa o modelo treinado mais recente.