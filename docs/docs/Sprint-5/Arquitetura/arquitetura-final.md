---
title: Visão geral da Arquitetura do sistema 
sidebar_position: 3
---

### Visão geral da Arquitetura do Sistema

#### Visão Geral

O sistema integra vários componentes para operar um robô de teleoperação equipado com câmera, LIDAR, e controle remoto via WebSockets. A arquitetura envolve uma série de nodes ROS para comunicação interna do robô, um backend em Express.js para processamento de dados e controle, um microserviço Python para inferência de imagem, e um frontend em React para interação com o usuário. Esta documentação fornece uma visão detalhada de como esses componentes se conectam e interagem, garantindo uma operação coordenada e eficiente do sistema.

#### Diagrama de Arquitetura

![diagrama-arquitetura](/img/diagrama-arquitetura.jpg)

#### Descrição dos Componentes

##### ROS Nodes

O sistema utiliza vários nodes ROS para diferentes funções essenciais. O node `bolin_camera` é responsável por ler a câmera do robô usando OpenCV e publicar o feed de vídeo no tópico `/camera_feed`. Este feed utiliza mensagens do tipo `CompressedImage`, com resolução de 640x480 pixels e taxa de aproximadamente 30 quadros por segundo, em formato JPEG.

O node `bolin_teleop` recebe comandos de velocidade angular e linear nos tópicos `/angular_speed` e `/linear_speed`, respectivamente. Ele remapeia esses comandos para os intervalos de velocidade adequados para o robô e publica no tópico `/cmd_vel`. Além disso, este node possui um serviço de segurança chamado `kill_robot_service`, que pode ser acionado para parar imediatamente o robô e interromper o processo de teleoperação em situações de emergência.

<div align="center">

![linear-angular](/img/linear-angular.avif)

_Representação da velocidade linear e angular_

</div>

O node `bolin_lidar` processa os dados do LIDAR, lendo as leituras no tópico `/scan` e publicando as distâncias dos obstáculos no tópico `/obstacle_distances`. Ele divide a leitura do LIDAR em oito quadrantes, cada um representando um lado do robô, e publica a distância em centímetros do obstáculo mais próximo em cada quadrante.

O metapackage `bolin` agrupa todos esses nodes e inclui um mock da bateria, usado para testes no simulador Webots. O pacote `bolin_bringup` fornece arquivos de lançamento para iniciar todos os nodes necessários. O arquivo `webots_launch.py` é usado para rodar no simulador, iniciando a câmera, teleop, LIDAR, mock da bateria e o próprio simulador. Já o arquivo `launch.py` é utilizado para rodar no robô real, iniciando a câmera, teleop, LIDAR e o launch file do Turtlebot.

##### Backend (Express.js)

O backend do sistema é implementado em Express.js e possui várias funções essenciais. O `Teleop Controller` é responsável por se subscrever no tópico `obstacle_distances` e publicar nos tópicos `angular_speed` e `linear_speed`. Ele integra um sistema de segurança que utiliza os dados do LIDAR para evitar que o robô se mova em direção a obstáculos. Este componente usa a biblioteca `roslib` para comunicação com os tópicos ROS usando Websockets disponibilizados pelo RosBridge.

O sistema de segurança integrado ao Teleop Controller utiliza os dados do LIDAR para tomar decisões em tempo real, garantindo que o robô não colida com obstáculos. Além disso, o backend implementa WebSockets para comunicação com o frontend, permitindo o envio de dados do LIDAR e a recepção de comandos de teleoperação de maneira eficiente e em tempo real.

O backend também expõe uma REST API para lidar com o gerenciamento de ordens de limpeza, examinações, reboilers, robôs e etc. Também conta com o microserviço de inferência de imagem, que recebe imagens capturadas pela câmera do robô e envia a informação de sujidade, obtida por um modelo de detecção de objetos.

A conexão com o banco de dados é gerida pelo Prisma ORM, que facilita o gerenciamento de ordens e o status do robô, integrando todas as informações necessárias para o funcionamento do sistema, facilitando a implementação de mecanismos de segurança contra ataques de SQL Injection.

##### Microservice (Python)

O microserviço de inferência de imagem é implementado em Python usando o microframework Flask. Este serviço recebe imagens do robô através do backend e retorna a sujeira presente nas imagens, utilizando um modelo de inferência pré-treinado. Este componente é essencial para a detecção de sujeira no ambiente, fornecendo dados importantes para a operação do robô.

##### Frontend (React)

O frontend é desenvolvido em React e se comunica com o backend usando WebSockets. Esta comunicação permite o envio de comandos de teleoperação e a recepção de dados do LIDAR, proporcionando uma interface de controle em tempo real para o operador do robô. O frontend também utiliza a biblioteca ROSLib para se conectar diretamente aos tópicos ROS com WebSockets, permitindo a visualização do feed da câmera e dos dados de bateria.

Além disso, o frontend realiza operações de gerenciamento, como a criação de ordens e a visualização do status do robô e suas operações, utilizando requisições HTTP para interagir com o backend.
