---
title: Back-End
sidebar_position: 3
---

Esta página documenta a arquitetura e implementação do Back-End do nosso sistema de controle do robô. O Back-End, desenvolvido em *JavaScript* utilizando o framework *Express*, é responsável por orquestrar a comunicação entre o Front-End e o robô através de ROS2 e Websockets. Exploraremos as tecnologias utilizadas, a estrutura de arquivos do projeto, e detalhes específicos sobre os componentes de teleoperação, câmera e lidar. Além disso, discutiremos considerações futuras para melhorar a performance e manutenibilidade do sistema.

## Tecnologias Utilizadas

- **JavaScript**
  - **[Express.js](https://expressjs.com/)**: Framework para construção de aplicações web, utilizado para criar o servidor HTTP.
- **ROS2 (Robot Operating System 2)**
  - [**rclnodejs**:](https://github.com/RobotWebTools/rclnodejs) Biblioteca que permite a comunicação com ROS2 utilizando JavaScript.
- **WebSockets**
  - [**ws**](https://github.com/websockets/ws): Biblioteca para implementar a comunicação em tempo real entre o servidor (Back-End) e o Front-End.

## Comunicação

### Comunicação com o Robô (ROS2)

O Back-End utiliza `rclnodejs` para se comunicar com o robô através de ROS2. Isso envolve a criação de nós ROS2 que publicam e subscrevem em tópicos, bem como a chamada de serviços.

### Comunicação com o Front-End (WebSockets)

A comunicação em tempo real com o Front-End é feita usando WebSockets, especificamente a biblioteca `ws`. Esta configuração permite que o Back-End envie e receba dados do Front-End de forma eficiente.

## Arquitetura e Estrutura de Arquivos

Para garantir a manutenibilidade e uma estrutura organizada, seguimos as melhores práticas de desenvolvimento. A estrutura de arquivos do projeto é a seguinte:

```
📦 src/backend
├── 📂 api
│   ├── 📂 controllers
│   │   ├── 📄 teleop-controller.js
│   │   ├── 📄 camera-controller.js
│   │   └── 📄 lidar-controller.js
│   ├── 📂 data
│   │   └── 📄 logs.json
│   ├── 📂 routes
│   │   ├── 📄 teleop-router.js
│   │   ├── 📄 camera-router.js
│   │   └── 📄 lidar-router.js
├── 📂 config
│   ├── 📄 default.json
│   └── 📄 express.js
├── 📄 package.json
└── 📄 server.js
```

### Descrição dos Diretórios e Arquivos

- **api**: Diretório principal da API.
  - **controllers**: Contém os controladores da aplicação, que lidam com a lógica de negócios e processamento de dados. Eles servem atualmente majoritariamente para abrir as conexões com o ROS e os websockets.
    - **teleop-controller.js**: Controlador responsável por gerenciar a teleoperação do robô.
    - **camera-controller.js**: Controlador responsável pela transmissão das imagens da câmera.
    - **lidar-controller.js**: Controlador responsável pela transmissão dos dados do lidar.
  - **data**: Diretório destinado ao armazenamento de dados. Atualmente, contém apenas um placeholder `logs.json`, mas futuramente será expandido para incluir interações com bancos de dados.
  - **routes**: Define as rotas da aplicação, ligando URLs a controladores específicos.
    - **teleop-router.js**: Rotas responsáveis pelas operações de teleoperação.
    - **camera-router.js**: Rotas responsáveis pela transmissão das imagens da câmera.
    - **lidar-router.js**: Rotas responsáveis pela transmissão dos dados do lidar.
- **config**: Configurações do servidor e outros parâmetros importantes.
  - **default.json**: Arquivo de configuração contendo parâmetros como host, port, e configurações de CORS.
  - **express.js**: Configurações específicas para o framework Express.
- **package.json**: Arquivo de configuração do Node.js que define as dependências e scripts do projeto.
- **server.js**: Arquivo principal que inicializa e configura o servidor Express.

Exemplo de configuração do `default.json`:

```json
{
  "server": {
    "host": "0.0.0.0",
    "port": 8000,
    "cors": {
      "origin": "*",
      "methods": "GET,HEAD,PUT,PATCH,POST,DELETE",
      "preflightContinue": false,
      "optionsSuccessStatus": 204
    },
    "teleop": {
      "port": 8001,
      "path": "/teleop"
    }
  }
}
```

## Teleop

A teleoperação é gerenciada por um WebSocket aberto na rota `/teleop`. Esta configuração permite a comunicação bidirecional entre o Back-End e o Front-End, essencial para controlar o robô em tempo real.

### Detalhes de Implementação

- **Recepção de Comandos**: O WebSocket na rota `/teleop` recebe comandos de velocidade linear e angular, além do comando `kill` para parar o robô.
- **Integração com ROS2**: Utilizando `rclnodejs` [rclnodejs](https://github.com/RobotWebTools/rclnodejs), criamos publishers para enviar as velocidades ao robô e um cliente para o `kill_service`, responsável por parar a teleoperação do robô.
- **Integração com websocket**: Para implementar o WebSocket, usamos a biblioteca [ws](https://github.com/websockets/ws), garantindo comunicação eficiente com o Front-End.

## Câmera

A funcionalidade da câmera é implementada de forma semelhante à teleoperação. Temos um nó ROS que se subscreve no tópico `camera_feed`, recebe cada frame, descompacta a imagem e publica no WebSocket aberto na rota `/camera_feed`. As imagens são enviadas ao Front-End em formato base64.

## Lidar

O nó lidar se subscreve no tópico `/scan` e processa os dados de distância recebidos. As informações são então publicadas no WebSocket, permitindo que o Front-End receba e processe as distâncias das extremidades em tempo real.

## Considerações e Próximos Passos

- **Isolamento do ROS2**: Consideramos usar [rosbridge](https://wiki.ros.org/rosbridge_suite) para rodar os WebSockets diretamente no robô, isolando o ROS2 do resto do sistema e garantindo melhores práticas. No entanto, isso pode afetar a performance e precisa ser testado.
- **Desempenho da Câmera**: Estamos estudando a possibilidade de escrever o nó da câmera em C++ para melhorar a performance, embora isso possa dificultar a manutenção devido à menor familiaridade com a linguagem.
- **cv_bridge**: Avaliamos o uso de [cv_bridge](https://wiki.ros.org/cv_bridge) para facilitar a transmissão das imagens.
