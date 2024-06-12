---
title: Robô
sidebar_position: 2
---

> Nosso robô chama "bolin" :)

Esta página fornece uma visão detalhada sobre os componentes e funcionalidades rodando diretamente no robô, todos desenvolvidos utilizando ROS2. Abordaremos os nodes principais como `bolin_teleop`, `bolin_camera` e `bolin_lidar`, explicando como cada um deles contribui para o controle do robô, aquisição de imagens e detecção de obstáculos. A documentação também inclui considerações sobre decisões de design e próximos passos para melhorias na performance e eficiência do sistema.

## Descrição Geral

Toda a comunicação e programação relacionadas ao robô são realizadas em [ROS2](https://index.ros.org/doc/ros2/). Rodamos vários nós para diferentes funcionalidades, como a câmera, a teleoperação e o lidar. Utilizamos um launchfile para inicializá-los.

## Nós em Execução

- **Nó da Câmera**
- **Nós de Teleop**
- **Nó do Lidar**

### Bolin_Teleop

O node `bolin_teleop` é responsável por ouvir os tópicos `linear_speed` e `angular_speed`, processar os valores recebidos (convertidos de porcentagens) e publicar um comando `Twist` no tópico `cmd_vel`, que controla o movimento do robô.

#### Configuração do Movimento

- Utilizamos o node do Turtlebot3 (`ros2 launch turtlebot3_bringup robot.launch.py`) para receber e interpretar os comandos `cmd_vel`.

#### Local de Execução

- Embora trate do movimento do robô, o node `bolin_teleop` é executado junto do servidor back-end para maximizar a performance, dado que a Raspberry Pi 4 usada no robô tem limitações de processamento.

### Bolin_Camera

O node `bolin_camera` utiliza OpenCV [OpenCV](https://opencv.org/) para capturar imagens da câmera. As imagens são comprimidas e enviadas no tópico `/camera_feed`. As imagens são enviadas em um spin em vez de um timer callback, como utilizado inicialmente, para melhorar a performance.

#### Transmissão de Imagens

- Avaliamos o uso de `cv_bridge` [cv_bridge](https://wiki.ros.org/cv_bridge) para facilitar a transmissão das imagens. As imagens são recebidas pelo back-end para processamento e envio ao Front-End. No entanto, considerando o tempo de entrega do projeto, foi decidido manter a transmissão de imagens diretamente pelo ROS2 e será considerado o teste na sprint seguinte.

- Também consideramos utilizar escrever o nó em C++ para melhorar a performance e diminuit a latência na transmissão. No entanto, após testes, percebemos que a diferença não era significativa. Segue abaixo os testes realizados:

##### Teste com C++
![Teste de Performance com C++](/img/teste-camera-cpp.jpeg)

##### Teste com Python
![Teste de Performance com Python](/img/teste-camera-python.jpeg)

Portanto, a partir dessa comparação, foi decidido que o nó continuaria sendo escrito em Python.

### Bolin_Lidar

O node lidar processa os dados recebidos do tópico `/scan`, dividindo-os em 8 direções principais.

<!-- Aqui vai uma imagem demonstrando quais são -->

#### Processamento de Dados

- Seleciona o menor valor de cada direção e publica no tópico `/lidar`.

#### Local de Execução

- Este node, embora relacionado ao robô, é executado no servidor back-end para melhorar a performance.
