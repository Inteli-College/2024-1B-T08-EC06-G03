---
title: Front-End
sidebar_position: 4
---

Esta página descreve o desenvolvimento e funcionamento do Front-End do nosso sistema de controle do robô, construído com React. O Front-End é responsável por fornecer uma interface de usuário intuitiva para a teleoperação do robô, incluindo controle via joystick e visualização em tempo real das imagens da câmera. Discutiremos as funcionalidades principais, a integração com o Back-End através de WebSockets, e os alertas de proximidade baseados nos dados do lidar. A documentação também aborda futuras melhorias planejadas para a interface do usuário.

## Tecnologias Utilizadas

### React
Optamos pelo React devido à sua eficiência na construção de interfaces de usuário dinâmicas e responsivas. Sua estrutura baseada em componentes facilita a reutilização de código e a manutenção da aplicação, além de proporcionar uma experiência de desenvolvimento ágil com seu ecossistema rico e suporte comunitário robusto.

### WebSockets
Utilizamos WebSockets para garantir comunicação em tempo real entre o Front-End e o Back-End. Esta escolha permite uma troca de dados contínua e bidirecional, essencial para o controle remoto do robô e a visualização de dados sensoriais em tempo real.

## Funcionalidades e Telas

### Tela de Teleop
A tela de teleop combina o controle do robô com a visualização da câmera, proporcionando ao usuário uma interface centralizada para todas as ações necessárias. A escolha de integrar o controle e a visualização na mesma tela visa minimizar o tempo de resposta do usuário e facilitar a tomada de decisões rápidas durante a operação do robô.

<div align="center">

*Tela de Teleoperação*

![Tela de Teleoperação](/img/front-controle.png)
*Fonte:* Elaborado pela equipe Rebólins

</div>

<div align="center">

*Tela de Teleoperação - Menu Lateral*

![Tela de Teleoperação](/img/front-controle-menu.png)
*Fonte:* Elaborado pela equipe Rebólins

</div>

### Transmissão de Imagem
A transmissão de imagem em tempo real permite ao usuário visualizar o ambiente em que o robô está, facilitando sua locomoção e manobras. Exibir a latência da transmissão fornece ao usuário uma noção da qualidade da conexão, essencial para ajustes de operação em ambientes com variabilidade de rede. O cálculo da latência foi realizado considerando a diferença de tempo de recebimento entre dois frames, garantindo precisão na medição.

### Joystick
Implementamos um joystick virtual que permite ao usuário controlar a velocidade linear e angular do robô. A escolha do joystick se deve à sua familiaridade e facilidade de uso em interfaces de controle remoto, proporcionando um controle intuitivo e preciso. Além disso, o controle é otimizado na utilização de dispositivos móveis.

### Botão de Kill
O botão de Kill aciona o kill_service no Back-End para parar imediatamente o robô em situações de emergência. Esta funcionalidade crítica foi projetada com destaque visual e fácil acesso para garantir a segurança durante a operação do robô.

### Lidar
O sistema de lidar alerta o usuário se o robô estiver muito próximo de um obstáculo. Esta funcionalidade de segurança é fundamental para evitar colisões e danos ao robô e ao ambiente, melhorando a confiança do usuário na operação remota.

## WebSockets

### Teleop WebSocket
Gerencia a comunicação do joystick e do kill_service. A escolha por WebSockets permite uma comunicação de baixa latência e alta frequência, essencial para o controle em tempo real.

### Camera WebSocket
Recebe e exibe as imagens da câmera do robô em tempo real, assegurando que o usuário tenha uma visão contínua e atualizada do ambiente.

### Lidar WebSocket
Recebe e exibe as informações do lidar em tempo real, proporcionando feedback imediato sobre a proximidade de obstáculos.

## Considerações e Próximos Passos

### Implementação da Interface de Ajuda
Iremos implementar uma interface para orientar o operados acerca das funcionalidades no display:

<div align="center">

*Tela de Teleoperação*

![Tela de Teleoperação](/img/ajuda2.png)


</div>
<div align="center">
*Fonte:* Elaborado pela equipe Rebólins
</div>

<div align="center">

*Tela de Teleoperação - Menu Lateral*

![Tela de Teleoperação](/img/ajuda-menu.png)

</div>
<div align="center">
*Fonte:* Elaborado pela equipe Rebólins
</div>


### Melhoria da Interface
Planejamos aprimorar a interface do usuário para torná-la mais intuitiva e responsiva. Isso inclui a revisão contínua do layout e a implementação de feedback visual mais claro para as ações do usuário.

### Implementação de Landing Page e Dashboards
Implementaremos uma landing page para que os usuários não caiam diretamente na interface de controle, melhorando assim a navegabilidade pelas diferentes funções da plataforma. Ademais, criaremos uma página que exibirá dashboards referentes aos dados coletados na operação do robô.

### Feedback do Usuário
Implementaremos modificações e ajustes conforme feedbacks do parceiro de projeto.

Conclusão
---

Ao longo do processo criativo, nossa prioridade foi criar uma interface que oferecesse controle preciso e feedback em tempo real, elementos essenciais para a operação segura e eficiente do robô. A escolha de tecnologias e a estrutura da interface foram guiadas pelo objetivo de proporcionar uma experiência de usuário robusta, intuitiva e confiável.
