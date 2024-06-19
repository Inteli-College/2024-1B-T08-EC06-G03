---
title: Frontend - Interface do Usuário
sidebar_position: 3
---

# Frontend - Interface do Usuário

## Contexto

Para controlar os procedimentos de inspeção do robô, isto é, a verificação do nível de sujidade antes e após a limpeza manual, é necessária uma interface que permita visualizar as diferentes inspeções que ocorrem, os reboilers e a frequência de limpeza, entre outras informações essenciais tanto à persona que irá controlar o robô quanto à equipe que irá visualizar e analisar os dados coletados. Dessa forma, foram idealizadas duas telas principais na interface gráfica: a tela de operação e a interface de controle dos dados.

## Tela de operação

A tela de operação é a interface que o operador irá utilizar enquanto controla o robô. Ela foi pensada para ter a maior simplicidade e intuitividade possível, de forma que o operador possa focar na inspeção do reboiler sem se preocupar com a interface ou comandos mais complexos. A tela de operação é composta por diversos elementos: 

- câmera: uma visualização da câmera do robô no centro;
- Joystick: controla o robô em diferentes angulações;
- Botão de kill: deve ser acionado em casos de emergência, que para o robô imediatamente e o impossibilita de se mover após o acionamento do botão
- Botão de câmera: aciona o modelo de visão computacional, processa o frame atual da câmera e retorna o resultado (sujo ou limpo) para o operador.

![lalala](/img/front-teleop.png)

## Tela de controle de processos

A tela de controle de processos é a interface que a gerência da manutenção poderá controlar os diferentes processos que irá utilizar para visualizar e analisar os dados coletados pelo robô. Ela foi pensada para ser mais complexa e detalhada,trazendo informações sobre cadas reboiler, quais os robôs cadastrados naquela unidade, quais procedimentos estão ativos e o histórico de limpezas, por exemplo. Para exibir essas diversas informações, a tela de controle de processos possui:

- Tabela de reboilers: exibe os reboilers cadastrados na unidade;
- Tabela de robôs: exibe os robôs cadastrados na unidade;
- Tabela de procedimentos: exibe o histórico de procedimentos na unidade;

![Tabela de reboilers]()

![Tabela de robôs]()

![Tabela de procedimentos]()

Essas tabelas são exibidas por meio de um tab system, onde o usuário pode selecionar qual tabela deseja visualizar e tem acesso a um clique em diferentes tabelas, facilitando a visualização.

Outro ponto é a seleção de unidade, exibida no centro superior da tela, que é o primeiro passo do usuário para verificar determinada informação, ele escolhe a unidade da qual ele quer ver as informações e a tela é atualizada com as informações da unidade selecionada.

![Seção da unidade]()

Existe também o botão de criar unidade, posicionado ao lado da seleção de unidade, assim como os botões de criar procedimentos, robôs e reboilers, permitindo o cadastro de novas informações. Ao clicar, um modal irá surgir para inserir as informações e cadastrar no banco de dados.

![Modal de criar]()

Voltando aos detalhes das tabelas, na tabela de procedimentos, é possível ver mais detalhes sobre um procedimento específico, como a data de início, data de fim, reboiler associado, robô associado e o andamento de cada etapa do procedimento (pré-limpeza ou pós-limpeza). Além disso, ao clicar sobre cada inspeção, é possível ver dados sobre aquela inspeção, ou seja, imagens tiradas do reboiler e quantos canos foram identificados sujos, fornecendo informações substanciais para análise da eficiência da limpeza, trazer reflexões sobre a frequência de limpeza e identificar possíveis problemas. Por exemplo, caso um reboiler necessite constantes limpezas, mas a inspeção após limpeza identifica que muitos canos continuam sujos, a equipe de manutenção pode provisionar uma limpeza mais detalhada ou verificação de funcionamento da máquina, mitigando possíveis falhas.

![detalhes do procedimento da tabela de procedimentos]()

Outra funcionalidade é o botão de retomada de sessão em inspeções que ainda estão ativas, de modo que, caos o operador do robô tenha interrompido a inspeção ou saia da página de controle, ele possa retomar. Assim, ele deve entrar na página de controle, ir no procedimento ativo, clicar no botão retomar sessão e ele será redirecionado para a página de controle novamente.

Portanto, a página de controles tem o objetivo de reunir todas as funcionalidades de gerência e controle de processo em uma tela só, facilitando a visualização e análise de dados, além de permitir o cadastro de novas informações.


