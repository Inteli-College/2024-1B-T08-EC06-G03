---
title: Visão Computacional
sidebar_position: 3
---

# Modelo de Visão Computacional - YOLO

Um dos requisitos para o funcionamento desse projeto é a detecção de resíduos dentro dos tubos, a fim de determinar o grau de impureza dentro dos reboilers. Para isso, utilizamos um modelo de [visão computacional](https://aws.amazon.com/pt/what-is/computer-vision/), o [YOLOv8](https://docs.ultralytics.com/models/yolov8/), que é voltado para a detecção de objetos em imagens em tempo real.

## Escolha de versão do modelo

No projeto, foi decidida a utilização da versão [YOLOv8](https://docs.ultralytics.com/models/yolov8/), apesar de existirem atualizações mais recentes, porque a instalação é facilitada (não há a necessidade de clonar um repositório, por exemplo, como a [YOLOv9](https://docs.ultralytics.com/models/yolov9/)). Além disso, a YOLOv8 possui mais exemplos encontrados online, facilitando o aprendizado dos desenvolvedores e implementação, já que a documentação é mais rica, o que não é o caso da versão [YOLOv10](https://docs.ultralytics.com/models/yolov10/), que foi lançada durante o desenvolvimento desse projeto, e ainda não tem muitos casos de uso.

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

### Métricas de Avaliação

1. **Precisão (Precision)**

    A precisão mede a proporção de verdadeiros positivos (detecções corretas de sujeira) sobre o total de detecções positivas (verdadeiros positivos + falsos positivos).

    - **Importância no Projeto:** Uma alta precisão significa que a maioria das detecções de sujeira são corretas. Isso é crucial porque:
        - **Cenário Realista:** Garante que o sistema não está identificando sujeira onde não existe, evitando alarmes falsos.
        - **Mitigação de Problemas de Vazão:** Detecções precisas permitem intervenções adequadas e pontuais, prevenindo problemas de vazão devido à sujeira nos tubos.
   
2. **Recall**

    O recall mede a proporção de verdadeiros positivos sobre o total de instâncias reais positivas (verdadeiros positivos + falsos negativos).

    - **Importância no Projeto:** Um alto recall indica que o modelo está capturando a maioria dos casos reais de sujeira. Isso é importante porque:
        - **Completa Detecção:** Assegura que a maior parte da sujeira é detectada, reduzindo a probabilidade de deixar resíduos não identificados que podem causar problemas de performance no futuro.
        - **Intervenção Adequada:** Garante que a manutenção pode ser realizada com base em uma detecção abrangente, melhorando a eficácia das limpezas e manutenções.

3. **mAP50**

    Mean Average Precision (mAP) ao usar um limiar de Intersection over Union (IoU) de 0.50. Isso significa que uma predição é considerada correta se a sobreposição (interseção) entre a caixa predita e a caixa real for de pelo menos 50%.
    
    - **Importâncias no Projeto:**
        - **Indicador de Desempenho Inicial:** Fornece uma medida clara de quão bem o modelo está detectando resíduos quando a exigência de sobreposição não é muito rigorosa (50% de sobreposição).
        - **Benchmarking:** É uma métrica padrão para comparar a performance inicial do modelo com outros modelos ou versões anteriores.
        - **Acurácia Prática:** Num cenário de inspeção de tubos, um mAP50 alto indica que o modelo é capaz de detectar resíduos de maneira robusta, mesmo com variações leves na precisão da localização.

4. **mAP50-95**

    Mean Average Precision calculada em múltiplos limiares de IoU, de 0.50 a 0.95 em incrementos de 0.05 (ou seja, IoU=0.50, 0.55, 0.60, ..., 0.95). Isso proporciona uma visão mais completa da performance do modelo em diferentes níveis de exigência de sobreposição.

    - **Importância no Projeto:**
        - **Indicador de Desempenho Inicial:** Fornece uma medida clara de quão bem o modelo está detectando resíduos quando a exigência de sobreposição não é muito rigorosa (50% de sobreposição).
        - **Benchmarking:** É uma métrica padrão para comparar a performance inicial do modelo com outros modelos ou versões anteriores.
        - **Acurácia Prática:** Num cenário de inspeção de tubos, um mAP50 alto indica que o modelo é capaz de detectar resíduos de maneira robusta, mesmo com variações leves na precisão da localização.

### Impacto das Métricas no Contexto de Negócio

1. **Confiabilidade e Eficiência Operacional**
    - **Precisão Alta:** Garante que os recursos não sejam desperdiçados em falsas detecções, permitindo uma manutenção mais eficiente e econômica.
    - **Recall Alto:** Assegura que a maioria das sujeiras sejam identificadas e tratadas, evitando problemas de performance e manutenção corretiva não planejada.

2. **Segurança e Qualidade**
    - **Detecções Confiáveis:** Reduz o risco de falhas catastróficas nos reboilers devido a acúmulo de sujeira não detectada.
    - **Qualidade dos Dados:** Proporciona dados precisos e completos para análises posteriores, suportando decisões informadas sobre a manutenção e operação dos reboilers.

3. **Decisões Estratégicas**
    - **Dados Verídicos:** Com métricas equilibradas, a empresa pode confiar nos dados para planejamento estratégico, como agendamento de manutenções preventivas e investimentos em melhorias.

Ao analisar essas métricas, a equipe técnica pode garantir que o modelo de detecção de sujeira nos reboilers está funcionando de maneira eficiente e eficaz, oferecendo suporte robusto para a tomada de decisões operacionais e estratégicas. A exemplo de umas das nossas personas, o **Operador de máquinas** Danillo Chrystian, que precisa de uma interface clara e intuitiva para operar o robô de inspeção, a precisão e o recall altos garantem que ele possa confiar nas detecções do modelo para agir de forma rápida e precisa em situações de emergência.

### Métricas de perda

As métricas de perda (ou loss) são fundamentais no treinamento de modelos de aprendizado de máquina, incluindo aqueles usados para detecção de objetos como o YOLOv8. Elas indicam o quanto o modelo está errando e ajudam a ajustar os pesos do modelo para melhorar sua performance. No contexto do seu projeto de detecção de resíduos em tubos de reboilers, as métricas de perda se relacionam da seguinte forma:

1. **Box Loss (Perda de Caixa)**

    A classification loss mede o erro na classificação dos objetos dentro das caixas delimitadoras. No projeto, refere-se à capacidade do modelo de distinguir corretamente entre tubo (tube) e sujeira (dirt).

    - **Localização:** O quão precisa é a posição da caixa predita.
    - **Dimensão:** O quão precisas são as dimensões (largura e altura) da caixa predita.

    **Importância no Projeto:**
    - **Precisão na Detecção de Resíduos:** Uma baixa box loss significa que o modelo está delimitando os resíduos dentro dos tubos de forma precisa. Isso é essencial para:
        - **Análises Detalhadas:** Permitir uma análise precisa da quantidade e localização da sujeira.
        - **Intervenção Eficiente:** Assegurar que as intervenções de limpeza ou manutenção sejam direcionadas para as áreas corretas.

2. **Classification Loss (Perda de Classificação)**

    A classification loss mede o erro na classificação dos objetos dentro das caixas delimitadoras. No projeto, refere-se à capacidade do modelo de distinguir corretamente entre tubo (tube) e sujeira (dirt).

    **Importância no Projeto:**
    - **Identificação Correta de Resíduos:** Uma baixa classification loss significa que o modelo está corretamente identificando sujeira versus tubo. Isso é crucial para:
        - **Decisões de Manutenção:** Garantir que as áreas identificadas para limpeza contenham realmente resíduos.
        - **Confiabilidade dos Dados:** Proporcionar dados confiáveis para análise posterior e para informar a equipe de manutenção.

3. **Distribution Focal Loss**

    A distribution focal loss é uma métrica que foca em melhorar a performance do modelo em classes difíceis de distinguir. Ela dá mais peso a erros em classes menos frequentes ou mais difíceis de classificar, ajudando a modelar melhor esses casos.

    **Importância no Projeto:**
    - **Melhoria em Casos Difíceis:** Uma baixa distribution focal loss indica que o modelo está melhorando na detecção de resíduos difíceis de identificar, que podem ser críticos para:
        - **Detecção de Pequenos Resíduos:** Assegurar que até mesmo pequenas quantidades de sujeira sejam detectadas, evitando problemas de performance.
        - **Consistência e Confiabilidade:** Aumentar a confiança no modelo ao garantir que ele é robusto o suficiente para lidar com todos os tipos de resíduos, independentemente da sua frequência ou dificuldade de detecção.

### Impacto das Métricas de Perda 

1. **Aprimoramento Contínuo:**
    - **Ajuste de Pesos:** Durante o treinamento, essas métricas de perda são usadas para ajustar os pesos do modelo, melhorando gradualmente sua precisão e recall.
    - **Otimização do Modelo:** Minimizar essas perdas resulta em um modelo mais eficiente e eficaz na detecção de resíduos nos tubos.

2. **Precisão e Confiabilidade:**
    - **Box Loss:** Reduzir a box loss melhora a precisão da localização dos resíduos, garantindo que a análise e a manutenção sejam direcionadas corretamente.
    - **Classification Loss:** Reduzir a classification loss assegura que a identificação de sujeira é confiável, evitando falsas classificações que poderiam levar a intervenções desnecessárias.
    - **Distribution Focal Loss:** Focar na redução dessa perda melhora a detecção em cenários difíceis, garantindo que mesmo as áreas mais desafiadoras são corretamente analisadas.

3. **Impacto na Manutenção e Operações:**
    - **Planejamento de Manutenção:** Com perdas reduzidas, o modelo fornece dados mais precisos, suportando um planejamento de manutenção mais eficiente e econômico.
    - **Qualidade dos Dados:** Métricas de perda reduzidas aumentam a qualidade e a confiabilidade dos dados coletados, permitindo uma análise mais precisa e decisões operacionais melhor informadas.

4. **Segurança e Performance:**
    - **Redução de Falhas:** Detecções precisas e confiáveis de resíduos ajudam a evitar falhas catastróficas nos reboilers, garantindo a segurança e a continuidade das operações.
    - **Otimização da Vazão:** Ao identificar corretamente a sujeira, o modelo ajuda a manter a vazão ideal dos tubos, prevenindo bloqueios e otimizações ineficazes.

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

Enfim, os resultados obtidos com o treinamento do modelo YOLOv8 são altamente satisfatórios, com altas pontuações em todas as métricas avaliadas. A precisão, recall, mAP50 e mAP50-95 atingiram valores próximos de 1, indicando que o modelo é altamente preciso e eficaz na detecção de objetos nas imagens. Além disso, as métricas de perda (box loss, classification loss e distribution focal loss) diminuíram significativamente ao longo do treinamento, sugerindo que o modelo está aprendendo a localizar e classificar os objetos com precisão.

### Comparação entre os modelos treinados

Antes de finalizar o treinamento do modelo, foi feito um treinamento com 5 épocas para avaliar seu desempenho inicial. Abaixo, é possível observar um gráfico com algumas das métricas utilizadas para medir o modelo de 100 épocas, porém com 5:

<div align="center">

**Métricas após treinamento de 5 épocas**

![Resultado](../../static/img/results_5e.png)

**Fonte:** Elaborado pelo modelo YOLOv8

</div>

Analisando o gráfico acima, é notável que, apesar de apresentar bons resultados, o modelo mostrava tendência a melhorar em todas as métricas, mas ainda não havia atingido o seu potencial máximo. Por exemplo, a precisão mostra um aumento repentino na última época, enquanto o recall apresenta uma leve queda, o que pode sugerir um equilíbrio entre precisão e recall que ainda precisava ser ajustado. Além disso, a melhoria nas métricas mAP50 e mAP50-95 sugere que o modelo poderia se tornar mais eficaz em detectar objetos com diferentes níveis de confiança.

## Validação e conclusão

O modelo YOLOv8 foi treinado com sucesso para detectar sujeira em canos de reboilers, atingindo altas pontuações em métricas seja em precisão, recall, mAP50 e mAP50-95 ao ser treinado com 100 épocas, sendo capaz de detectar todos os objetos presentes nas imagens com alta precisão.

Abaixo é possível observar um exemplo de uma imagem de teste com a detecção de sujeira realizada pelo modelo:

<div align="center">

**Primeiro exemplo de detecção de sujeira em canos**

![Detecção de sujeira](../../static/img/val_batch0_pred.jpg)

**Fonte:** Elaborado pelo modelo YOLOv8


</div>

Observando esta imagem, pode-se perceber uma alta confiança na detecção de tubos (1.0) e uma boa confiança na detecção de sujeira (0.8-0.9) que indicam que o modelo está funcionando bem. Ainda considerando estes resultados, é possível afirmar que um operador poderia usar essas detecções para identificar rapidamente quais tubos precisam de limpeza ou manutenção, com base nas detecções de sujeira. Além disso, é importante afirmar que o modelo pode ser facilmente melhorado com mais dados e um aumento das épocas de treinamento.

<div align="center">

**Segundo exemplo de detecção de sujeira em canos**

![Detecção de sujeira](../../static/img/train_batch2.jpg)

**Fonte:** Elaborado pelo modelo YOLOv8

</div>

Nesta segunda imagem, é possível notar que o modelo foi capaz de detectar tanto os tubos (representados por 0) quanto a sujeira (representado por 1) presente neles em diferentes posições e ângulos, isto mostra a versatilidade do modelo em realizar detecções em cenários distintos e específicos.

Concluindo, é possível dizer que estas montagem de imagens validando o modelo YOLO no contexto do projeto mostram um bom desempenho não apenas na detecção de tubos e resíduos como na localização dos memsmos, fornecendo uma base sólida para intervenções de manutenção baseadas em detecções automatizadas. A confiança nas detecções e a categorização clara das imagens ajudam a garantir que o sistema seja prático e eficiente para uso em cenários reais.

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