---
title: Metodologia
sidebar_position: 1
---

## Movimentação do robô

O objetivo dessa sprint foi o desenvolvimento de um sistema (nesse caso, uma aplicação web) que é capaz de se comunicar com o robô e fazê-lo se mover. 

Para tal, primeiramente foi necessário realiazar a configuração do Turtlebot conforme é explicado [aqui](https://rmnicola.github.io/m6-ec-encontros/setupturtle#1-instalando-o-sistema-operacional-no-raspberry-pi). Ao finalizar essa configuração, é possível acessar o terminal do robô e mandá-lo executar algumas coisas por ssh usando o comando `ssh -p <porta> <nome de usuário>@<endereço IP>` e digitando a senha. Para realizar tal, ambos o robô e o computador precisam estar na mesma rede.

Com o terminal do robô aberto no computador, só é preciso fazer uma coisa: colocar ele pra rodar o comando `ros2 launch turtlebot3_bringup robot.launch.py`. Esse é um comando padrão que vem ao baixar o turtlebot3, que irá inicializar e fazer com que o robô "escute" os comandos que forem publicados na mesma rede.

Outra coisa configurada ainda no terminal do robô é o *ROS_DOMAIN_ID*, que é uma variável de ambiente que permite que só os que estão no mesmo domínio possam se comunicar. Então, em outra aba do terminal, foram executados os comandos `echo 'export ROS_DOMAIN_ID= <Valor-numérico-de-0-a-232> >> ~/.bashrc` e `source ~/.bashrc`. Nesse caso, o valor numérico escolhido foi o 231. 

O mesmo foi executado no terminal do computador, e, partir de agora, tudo é executado no mesmo.

Após isso, foi criado um workspace para acomodar o pacote que se chama robot_navigation, onde estão os scripts que mandam as informações para o robô. Para isso, foi utilizado o seguinte passo a passo:

- `mkdir -p workspace/src`
- `cd workspace/src`
- `ros2 pkg create --build-type ament_python robot_navigation`

Depois, algumas mudanças foram necessárias, como por exemplo editar o arquivo *package.xml* e adicionar as depenências de execução do rclpy e std_msgs. Além disso, também foi preciso mexer no arquivo *setup.py*, na parte de **entry_points**, adicionando na lista (que até então ainda estava vazia) a estrutura `"< nome do comando > = < nome do pacote > . < nome do arquivo executável > : < função principal, nesse caso a main >`.

Após essas mudanças, foi utilizado um script para compilar, executando `source run.sh`, finalizando assim a estruturação do workspace e do pacote.

Como foi citado, para definir um *entry_point*, ou seja, o que é executado ao rodar o pacote, foi necessário citar um arquivo executável e uma função dentro dele. Esse arquivo, nesse caso, encontra-se em `src/workspace/src/robot_navigation`, e se chama **bot.py**.

Foi nesse arquivo que foi definido o *publisher*, ou seja, os inputs que estão sendo enviados, e que serão "ouvidos" pelo robô, que é nosso *subscriber*.

Esse script está funcionando como um backend da nossa aplicação web, de forma que os componentes do frontend enviam um parâmetro para nosso publisher (bot.py) por meio de uma rota http, mandando assim o comando para o robô.

## Interface de Usuário

A interface de usuário "Central de Controle Desencana!" foi criada para facilitar a interação entre os operadores e o sistema de controle do robô. Esta interface intuitiva e de fácil manuseio permite que o operador de máquinas controle o robô remotamente com eficiência, e pode ser vista na imagem abaixo:

<div align="center">

**Interface de controle**

![Central de controle Desencana!](../../static/img/interface_principal.png)

**Fonte:** Elaborado pela equipe Rebólins

</div>

No canto superior esquerdo da tela, o status atual do robô é claramente exibido, podendo estar "Desligado" — indicando que o robô não está em operação — ou "Ligado", que mostra que o robô está ativo. A interface principal é organizada com dois botões proeminentes: um botão cinza e inativo à esquerda para ligar o robô, e um botão vermelho à direita, que funciona como um botão de emergência para interromper as operações do robô se necessário, com sua cor destacando sua importância.

No centro da tela, há uma área dedicada para a visualização da câmera do robô, permitindo que o operador monitore as ações do robô em tempo real. Instruções logo abaixo mostram como mover o robô usando as setas do teclado, oferecendo controle remoto efetivo e direto.

A parte inferior da tela reconhece o "grupo Rebôlins Inteli - Instituto de Tecnologia e Liderança" como os desenvolvedores da interface, destacando a colaboração na criação desta ferramenta essencial. A combinação de simplicidade e funcionalidade faz desta interface uma solução prática e acessível para a gestão remota do robô.

Por fim, um ícone de ponto de interrogação no canto superior direito serve como um atalho para o menu de ajuda, onde os usuários podem encontrar informações adicionais sobre a operação da interface e do robô.

A interface de usuário foi desenvolvida utilizando um html e css simples, com o objetivo de criar uma interface intuitiva e de fácil manuseio. O código foi organizado de forma a facilitar a manutenção e a adição de novas funcionalidades no futuro. Além disso, foi utilizado o javascript para a implementação de funcionalidades interativas, como o controle remoto do robô através de teclas do teclado.