---
title: Metodologia
sidebar_position: 1
---
# Metodologia da Sprint 2

A metodologia de projeto utilizada nesta Sprint 2 se concentrou na documentação meticulosa e no registro de cada etapa do desenvolvimento, aspectos cruciais para a execução bem-sucedida de uma metodologia ágil. Essa abordagem iterativa é fundamental porque facilita a adaptação e revisão do projeto conforme novas informações e desafios surgem ao longo da sprint. Nesta fase, o foco foi estabelecer e programar os movimentos básicos do robô TurtleBot3, teleoperado  por uma interface gráfica simples em uma rede local, permitindo uma integração contínua e ajustes rápidos, essenciais para o sucesso em um ambiente de desenvolvimento ágil.

## Movimentação do robô

A movimentação teleoperada do robô TurtleBot3 constitui a base para o desenvolvimento do projeto, pois é o primeiro passo para uma interação efetiva entre o usuário e o robô em um ambiente controlado. Esta sprint focou no desenvolvimento de uma aplicação web capaz de se comunicar com o robô e controlar seus movimentos básicos. A aplicação foi projetada para enviar comandos ao robô, que, após configurado conforme instruções disponíveis [aqui](https://rmnicola.github.io/m6-ec-encontros/setupturtle#1-instalando-o-sistema-operacional-no-raspberry-pi), podia ser acessado e controlado via SSH.

Para tal, primeiramente foi necessário realizar a configuração do Turtlebot conforme é explicado [aqui](https://rmnicola.github.io/m6-ec-encontros/setupturtle#1-instalando-o-sistema-operacional-no-raspberry-pi). Ao finalizar essa configuração, é possível acessar o terminal do robô e mandá-lo executar algumas coisas por ssh usando o comando  `ssh -p <porta> <nome de usuário>@<endereço IP>` e digitando a senha. Para realizar tal, ambos o robô e o computador precisam estar na mesma rede.

Com o terminal do robô aberto no computador, deve-se executar o  comando `ros2 launch turtlebot3_bringup robot.launch.py` nele. Esse é um comando padrão que vem ao baixar o turtlebot3, que irá inicializar e fazer com que o robô "escute" os comandos que forem publicados na mesma rede. Com isso, será possível realizar a comunicação via tópico e nós com o robô. Para saber mais sobre a comunicação, visualizar esse [link](https://rmnicola.github.io/m6-ec-encontros/ros1) 

Além disso, deve ser configurado, ainda no terminal do robô, o *ROS_DOMAIN_ID*, o qual é uma variável de ambiente que permite a comunicação entre dispositivos apenas para os que possuem o mesmo domínio. Então, em outra aba do terminal, foram executados os comandos `echo 'export ROS_DOMAIN_ID=<Valor-numérico-de-0-a-232> >> ~/.bashrc` e `source ~/.bashrc`. Nesse caso, o valor numérico escolhido foi o 231, sendo substituído onde está escrito `<Valor-numérico-de-0-a-232>`. O mesmo comando foi executado no terminal do computador, e, partir de agora, tudo é executado no mesmo.

Após isso, foi criado um workspace para acomodar o pacote que se chama robot_navigation, onde estão os scripts que mandam as informações para o robô. Para isso, foi utilizado o seguinte passo a passo:

- `mkdir -p workspace/src`
- `cd workspace/src`
- `ros2 pkg create --build-type ament_python robot_navigation`

Depois, algumas mudanças foram necessárias, como por exemplo editar o arquivo *package.xml*, alterando a descrição, licença, distribuidor e versão. Ademais, foi necessário adicionar essas linhas de código: `<exec_depend>rclpy</exec_depend> <exec_depend>std_msgs</exec_depend>`, as quais são depenências de execução do rclpy e std_msgs. Além disso, também foi preciso modificar o arquivo *setup.py*, na parte de **entry_points**, adicionando na lista (que até então ainda estava vazia) a estrutura `"< nome do comando > = < nome do pacote > . < nome do arquivo executável > : < função principal, nesse caso a main >`.

Após essas mudanças, criou-se um script chamado de `run.sh`, para rodar o projeto, o qual é responsável por verificar a existência do Flask, pacote Python necessário para rodar a aplicação. Sendo assim, o script verifica a existência dele na máquina e adiciona ao ROS, o qual não possui esse pacote. Por fim, é realizada a build do pacote ROS e sua execução. Para que o script pudesse ser rodado, deve-se executar o comando `chmod +x filename.sh`, para permitir ser um executável. Por fim, ele é executado  `./run.sh`, estando dentro da pasta **workspace**, assim finalizando assim a estruturação do workspace e do pacote.

Como foi citado, para definir um entry_point, ou seja, o que é executado ao rodar o pacote, foi necessário citar um arquivo executável e uma função dentro dele. Esse arquivo, nesse caso, encontra-se em src/workspace/src/robot_navigation, e se chama bot.py.

Foi nesse arquivo que foi definido o publisher, ou seja, os inputs que estão sendo enviados, e que serão "ouvidos" pelo robô, que é nosso subscriber.

A interface gráfica controlada por botões na tela permite que sinais sejam enviados para o Backend em Flask, uma escolha baseada na familiaridade do grupo e na facilidade de implementação com a programação dos pacotes ROS utilizados. O backend, por sua vez, aciona funções publisher que enviam um tópico para o subscriber rodando no robô, que executa os movimentos requisitados. Atualmente, o robô realiza movimentos como andar para frente, para trás, virar à esquerda, à direita, girar em torno do próprio eixo e fazer curvas suaves. Há também uma função de emergência que interrompe todas as operações para garantir segurança (kill button).

Para mais detalhes do código da aplicação, acesse [aqui.](https://github.com/Inteli-College/2024-1B-T08-EC06-G03/blob/main/src/workspace/src/robot_navigation/robot_navigation/bot.py)

## Interface de Usuário

Pensando no melhor desenvolvimento das funcionalidades de movimentação e comunicação com o robô, foi decidido desenvolver uma interface gráfica mais simples, se assemelhando a um wireframe, desenvolvido com HTML e CSS simples. Essas tecnologias foram escolhidas pela simplicidade de uso e implementação.

Houve a possibilidade de fazer interface por linha de comando, no entanto, foi escolhida uma interface gráfica, ainda que simples, e foram adicionadas as funcionalidades previstas no wireframe, desenvolvido no Figma(link do Figma), de modo a validar a usabilidade com o parceiro e as funcionalidades precistas e esperadas.

<div align="center">

**Interface de controle**

![Central de controle Desencana!](/img/interface-principal.png)

**Fonte:** Elaborado pela equipe Rebólins

</div>

No canto superior esquerdo da tela, o status atual do robô é claramente exibido, podendo estar "Desligado" — indicando que o robô não está em operação — ou "Ligado", que mostra que o robô está ativo. A interface principal é organizada com dois botões proeminentes: um botão cinza e inativo à esquerda para ligar o robô, e um botão vermelho à direita, que funciona como um botão de emergência para interromper as operações do robô se necessário, com sua cor destacando sua importância.

No centro da tela, há uma área dedicada para a visualização da câmera do robô, permitindo que o operador monitore as ações do robô em tempo real. Instruções logo abaixo mostram como mover o robô usando as setas do teclado, oferecendo controle remoto efetivo e direto.

A parte inferior da tela reconhece o "grupo Rebôlins Inteli - Instituto de Tecnologia e Liderança" como os desenvolvedores da interface, destacando a colaboração na criação desta ferramenta essencial. A combinação de simplicidade e funcionalidade faz desta interface uma solução prática e acessível para a gestão remota do robô.

Por fim, um ícone de ponto de interrogação no canto superior direito serve como um atalho para o menu de ajuda, onde os usuários podem encontrar informações adicionais sobre a operação da interface e do robô.

A interface de usuário foi desenvolvida utilizando um html e css simples, visando criar uma interface intuitiva e de fácil manuseio. O código foi organizado para facilitar a manutenção e a adição de novas funcionalidades no futuro. Além disso, foi utilizado o javascript para a implementação de funcionalidades interativas, como o controle remoto do robô por meio das teclas do teclado.