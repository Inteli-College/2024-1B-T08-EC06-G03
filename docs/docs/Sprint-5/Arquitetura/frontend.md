---
title: Frontend - Interface do Usuário
sidebar_position: 4
---

# Frontend - Interface do Usuário

# Versão inicial da interface

<div align="center">

**Interface de controle - V0**

![Central de controle Desencana!](/img/interface-principal.png)

</div>

A princípio, a interface de usuário era apenas uma página de teleoperação, feita com html e css simples. Havia um botão para interromper o funcionamento do robô, um para ligá-lo e a movimentação não era feita por meio da interface, e sim pelas teclas do teclado, não sendo acessível para aparelhos como celular ou tablet.
Com o decorrer das sprints, o projeto foi tomando forma e ganhando complexidade, conforme novas _features_ foram implementadas. Dessa forma, a interface do usuaário acabou sendo iterada várias vezes antes da versão final que será apresentada abaixo.

## Wireframe da versão final

A versão inicial da interface não contemplava as [user stories 2 e 3 do Jairo](../../Sprint-1/Design/personas.md), pois não havia a parte de visualização e validação de dados. Por essa razão, foi primeiramente esboçado um wireframe, o qual é uma visualização de baixa fidelidade do que era planejado ser implementado.

<div align="center">

**Wireframe da tela de acompanhamento de sessões**

![Wireframe 1](/img/wireframe-1.jpg)

</div>

<div align="center">

**Wireframe da tela de acompanhamento de sessões**

![Wireframe 2](/img/wireframe-2.jpg)

</div>

Como é possível observar, foi esboçado como funcionaria o fluxo de busca das sessões de inspeção realizadas pelo robô, considerando a unidade, os reboilers e os robôs atuantes, além de pensar na lógica de como a tela de teleoperação seria acessada (e foi estabelecido que ela só seria acessada após a identificação da sessão, mas isso será explicado melhor mais à frente). 

# Tecnologias utilizadas

Para a escolha das tecnologias utilizadas, as variáveis consideradas foram: facilidade de implementação e conhecimento prévio do time, facilidade de integração com o backend e a possibilidade de usar componentes pré-prontos. Pensando nesses aspectos, a decisão final foi utilizar o _build tool_ [Vite](https://vitejs.dev/) que os desenvolvedores já possuem uma maior familiaridade, o que facilitou a implementação e acelerou o tempo de desenvolvimento. Outro ponto importante é que há muitas bibliotecas disponíveis, como o [Tailwind](https://tailwindcss.com/docs/installation)  e o [ShadCN UI](https://ui.shadcn.com/), que possuem várias ferramentas que tornaram a estilização e criação dos componentes da interface muito mais efetivos.


# Interface Final

Para controlar os procedimentos de inspeção do robô, isto é, a verificação do nível de sujidade antes e após a limpeza manual, é necessária uma interface que permita visualizar as diferentes inspeções que ocorrem, os reboilers e a frequência de limpeza, entre outras informações essenciais tanto à pessoa que irá controlar o robô quanto à equipe que irá visualizar e analisar os dados coletados. Dessa forma, foram idealizadas duas telas principais na interface gráfica: a tela de operação e a interface de controle dos dados.

## Tela de operação

A tela de operação é a interface que o operador irá utilizar enquanto controla o robô. Ela foi pensada para ter a maior simplicidade e intuitividade possível, de forma que o operador possa focar na inspeção do reboiler sem se preocupar com a interface ou comandos mais complexos. A tela de operação é composta por diversos elementos: 

- câmera: uma visualização da câmera do robô no centro;
- Joystick: controla o robô em diferentes angulações;
- Botão de kill: deve ser acionado em casos de emergência, que para o robô imediatamente e o impossibilita de se mover após o acionamento do botão
- Botão de câmera: aciona o modelo de visão computacional, processa o frame atual da câmera e retorna o resultado (sujo ou limpo) para o operador.
- Bateria: Permite o acompanhamento da porcentagem de bateria do robô.

<div align="center">

**Tela de controle**

![Tela de controle](/img/front-teleop.png)

**Fonte:** Elaborado pela equipe Rebólins

</div>

Por fim, cada botão foi posicionado com o intuito de facilitar o controle do operador. Já que essa tela foi projetada para ser utilizada em uma versão mobile, com o tablet ou o celular, por exemplo, os botões ficaram nas áreas mais extremas, com maior facilidade de controle das mãos e pilotagem, além da câmera, estar no centro, facilitando o controle do robô pela interface.

## Tela de controle de processos

A tela de controle de processos é a interface que a gerência da manutenção poderá controlar os diferentes processos que irá utilizar para visualizar e analisar os dados coletados pelo robô. Ela foi pensada para ser mais complexa e detalhada, trazendo informações sobre cadas reboiler, quais os robôs cadastrados naquela unidade, quais procedimentos estão ativos e o histórico de limpezas, por exemplo. Para exibir essas diversas informações, a tela de controle de processos possui:

- Tabela de procedimentos: exibe o histórico de procedimentos na unidade;
- Tabela de robôs: exibe os robôs cadastrados na unidade;
- Tabela de reboilers: exibe os reboilers cadastrados na unidade;


### Tabela de procedimentos cadastrados
![Tabela de procedimentos](/img/procedimentos-cadastrados.png)

### Tabela de robôs cadastrados
![Tabela de robôs](/img/robos-cadastrados.png)

### Tabela de reboilers cadastrados
![Tabela de reboilers](/img/reboilers-cadastrados.png)


Essas tabelas são exibidas por meio de um tab system, onde o usuário pode selecionar qual tabela deseja visualizar e tem acesso a um clique em diferentes tabelas, facilitando a visualização.

Outro ponto é a seleção de unidade, exibida no centro superior da tela, que é o primeiro passo do usuário para verificar determinada informação, ele escolhe a unidade da qual ele quer ver as informações e a tela é atualizada com as informações da unidade selecionada.

### Seção da unidade - dropdown
![Seção da unidade](/img/dropdown-unit.png)

Existe também o botão de criar unidade, posicionado ao lado da seleção de unidade, assim como os botões de criar procedimentos, robôs e reboilers, permitindo o cadastro de novas informações. Ao clicar, um modal irá surgir para inserir as informações e cadastrar no banco de dados.

### Cadastrar robô
![Modal de criar](/img/apelido-robo.png)

Voltando aos detalhes das tabelas, na tabela de procedimentos, é possível ver mais detalhes sobre um procedimento específico, como a data de início, data de fim, reboiler associado, robô associado e o andamento de cada etapa do procedimento (pré-limpeza ou pós-limpeza). Além disso, ao clicar sobre cada inspeção, é possível ver o detalhamento da mesma, ou seja, imagens tiradas do reboiler e quantos canos foram identificados sujos, fornecendo informações substanciais para análise da eficiência da limpeza, trazer reflexões sobre a frequência de limpeza e identificar possíveis problemas. Por exemplo, caso um reboiler necessite constantes limpezas, mas a inspeção após limpeza identifica que muitos canos continuam sujos, a equipe de manutenção pode provisionar uma limpeza mais detalhada ou verificação de funcionamento da máquina, mitigando possíveis falhas.

### Cadastrar procedimento
![Iniciar novo procedimento](/img/cadastrar-procedimento.png)

Para iniciar um novo procedimento, que engloba o funcionamento do robô pré e pós-limpeza, é possível clicar no botão presente acima das tabelas no lado direito, escrito "cadastrar procedimento". Ao apertar esse botão, deve-se selecionar o robô e o reboiler já previamente cadastrados.

### Procedimento e informações sobre as inspeções

Outra funcionalidade é o botão de retomada de sessão em inspeções que continuam ativas, de modo que, caso o operador do robô tenha interrompido a inspeção ou saia da página de controle, ele possa retomar. Assim, ele deve entrar na página de controle, ir ao procedimento ativo, clicar no botão retomar sessão e ele será redirecionado para a página de controle novamente.

Portanto, a página de controles tem o objetivo de reunir todas as funcionalidades de gerência e controle de processo em uma tela só, facilitando a visualização e análise de dados, além de permitir o cadastro de novas informações.

## Conclusão

Portanto, as telas foram pensadas a partir da necessidade de cada persona e o principal foco foi trazer as funcionalidades de forma intuitiva, por meio de ícones, palavras-chave e botões acessíveis, além de possibilitar a melhor experiência do usuário com a interface. A tela de operação foi pensada para ser simples e direta, enquanto a tela de controle de processos foi pensada para ser mais detalhada e complexa, trazendo informações relevantes para a gerência da manutenção.

