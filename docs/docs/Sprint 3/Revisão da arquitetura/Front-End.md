---
title: Front-End
sidebar_position: 4
---

Esta página descreve o desenvolvimento e funcionamento do Front-End do nosso sistema de controle do robô, construído com React. O Front-End é responsável por fornecer uma interface de usuário intuitiva para a teleoperação do robô, incluindo controle via joystick e visualização em tempo real das imagens da câmera. Discutiremos as funcionalidades principais, a integração com o Back-End através de WebSockets, e os alertas de proximidade baseados nos dados do lidar. A documentação também aborda futuras melhorias planejadas para a interface do usuário.

## Tecnologias Utilizadas

- [**React**](https://react.dev/): Biblioteca JavaScript para construção de interfaces de usuário.
- [**WebSockets**](https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API): Utilizados para comunicação em tempo real com o back-end.

## Funcionalidades e Telas

A tela de teleop combina controle do robô com visualização da câmera. Nela, o usuário pode interagir com o robô em tempo real.
![Tela de teleop](/img/screenshot-frontend-teleop.png)

### Transmissão de imagem

A transmissão de imagem é feita em tempo real, permitindo ao usuário visualizar o ambiente em que o robô estar e facilitar sua locomoção. Também é exibida a latência da transmissão para que o usuário possa ter uma noção da qualidade da conexão. O cálculo da latência foi realizado considerando a diferença de tempo de recebimento entre dois frames. 

#### Joystick

- Permite ao usuário controlar a velocidade linear e angular do robô.

#### Botão de Kill

- Aciona o `kill_service` no back-end para parar o robô.

#### Lidar

- Alerta o usuário se o robô estiver muito próximo de um obstáculo.

#### WebSockets

- **Teleop WebSocket**: Gerencia a comunicação do joystick e do `kill_service` [ws](https://github.com/websockets/ws).
- **Camera WebSocket**: Recebe e exibe as imagens da câmera em tempo
- **Lidar WebSocket**: Recebe e exibe as informações do lidar em tempo real.

### Considerações e Próximos Passos

- **Melhoria da Interface**: Planejamos aprimorar a interface do usuário para torná-la mais intuitiva e responsiva.
- **Feedback do Usuário**: Implementaremos um sistema de feedback para entender melhor as necessidades dos usuários e melhorar a aplicação.
