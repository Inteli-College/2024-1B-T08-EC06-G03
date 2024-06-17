---
title: Revisão da arquitetura do sistema
sidebar_position: 1
---

## Introdução

Nesta seção, fornecemos uma visão abrangente da arquitetura de solução do nosso sistema de controle de robôs. A arquitetura é dividida em três principais subsistemas: Back-End, Robô e Front-End. Cada subsistema desempenha um papel crucial na comunicação e operação do robô.


- O [**Robô**](./robo) executa nós específicos para controle de movimento, aquisição de imagens e detecção de obstáculos, todos desenvolvidos em ROS2.
- O [**Back-End**](./back-end) orquestra a comunicação entre o Front-End e o Robô, utilizando tecnologias como ROS2 e WebSockets.

- O [**Front-End**](./front-end) fornece uma interface intuitiva para os usuários controlarem o robô e visualizarem informações em tempo real.

## Diagrama de Arquitetura

O diagrama abaixo representa a comunicação entre os subsistemas Back-End, Robô e Front-End, destacando os componentes de teleoperação, câmera e lidar. Caso queira ter uma melhor visualização do diagrama abaixo, também pode vê-lo [aqui](https://drive.google.com/file/d/1H0yqpZD8s6u_Ah7Jclvx53MnB_qDc-Hp/view?usp=sharing).

<div align="center">

**Arquitetura**

![Diagrama de Arquitetura](/img/diagrama-de-arquitetura.png)

**Fonte:** Elaborado pela equipe Rebólins

</div>

# Instruções de execução

Para executar o projeto, siga as instruções abaixo:

Clone o repositório:

```bash
git clone git@github.com:Inteli-College/2024-1B-T08-EC06-G03.git
cd 2024-T0008-EC06-G03/
```

## Robô

1. Instale o [ROS2](https://docs.ros.org/en/humble/Installation.html)

2. Acesse a pasta do robô

```bash
cd src/bolin/
```

3. Execute o robô

```bash
ros2 run bolin bolin

ros2 run  bolin_lidar bolin_lidar

ros2 run bolin_camera camera
```

## Back-End

1. Instale o [Node.js](https://nodejs.org/en/download/)

2. Acesse a pasta do Back-End

```bash
cd src/backend/
```

3. Instale as dependências

```bash
npm install
```

4. Execute o Back-End

```bash
npm start
```

## Front-End

1. Instale o [Node.js](https://nodejs.org/en/download/)

2. Acesse a pasta do Front-End

```bash
cd src/frontend/
```

3. Instale as dependências

```bash
npm install
```

4. Execute o Front-End

```bash
npm run dev
```

5. Acesse [http://localhost:5173](http://localhost:5173) no seu navegador.
