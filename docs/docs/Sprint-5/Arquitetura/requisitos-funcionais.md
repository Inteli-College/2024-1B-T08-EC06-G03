---
title: Requisitos funcionais
sidebar_position: 1
---

## Requisitos Funcionais

A seção de Requisitos Funcionais detalha as especificações cruciais que o sistema de robô teleoperado deve cumprir para realizar a inspeção de canos de reboilers de forma eficaz e eficiente. Esses requisitos são fundamentais para garantir que o robô atenda às necessidades de operação e manutenção definidas pelas personas do projeto, Danillo Chrystian Carvalho, o operador de máquinas, e Jairo Gomez Santos, o engenheiro de manutenção. Aqui, cada funcionalidade que o robô deve possuir é descrita claramente, incluindo as capacidades de navegação, processamento de dados, comunicação e segurança. Este documento serve como uma base para o desenvolvimento e teste do sistema, assegurando que todas as características do robô sejam projetadas para maximizar a usabilidade, a precisão na detecção de condições dos canos, e a facilidade de manutenção. Além disso, esta seção orienta a integração e a interação entre os componentes do sistema, garantindo que o robô funcione de    acordo com as expectativas dos usuários finais.

| Requisito | Descrição | [User story relacionada](Sprint-1/Design/personas.md) | Critérios de aceitação | Caso de teste |
|-----------|-----------|-----------------------|------------------------|---------------|
| RF01      | A interface de usuário permite que o robô seja controlado de maneira teleoperada | User story 1 do Danillo | O sistema apresenta acompanhamento das imagens enviadas pela câmera em tempo real, além de contar com uma interface de controle do robô | Verificação da transmissão de imagem em tempo real e do funcionamento do controle|
| RF02      | É possível acessar dados sobre uma inspeção | User story 2 do Jairo | É possível visualizar numa tabela qual a porcentagem de canos sujos do reboiler em uma inspeção | Visualizar a tela de visualização de dados|
| RF03      | O operador deve ser capaz de cancelar a inspeção a qualquer momento, de modo que o robô pare sua ação em no máximo 10 segundos. | User story 4 do Danillo Chrystian | O robô para sua ação em no máximo 10 segundos quando a inspeção é cancelada. | Pressionar o botão de cancelar durante a inspeção e verificar se o robô para sua ação em no máximo 10 segundos. |
| RF04      | Deve ser possível saber a proximidade do robô em relação às paredes, a fim de ser possível parar o robô antes de uma colisão| User story 3 do Jairo Santos. | A interface possui um aviso indicando a aproximação de um obstáculo | Aproximar o robô de um obstáculo e visualizar o sistema de alarme na tela|
| RF05      | O sistema deve processar as imagens capturadas para identificar sujeira ou resíduos, utilizando algoritmos de visão computacional. | User story 2 do Jairo Santos. | O sistema processa as imagens capturadas para identificar sujeira ou resíduos. | Capturar imagens. Verificar se o sistema identifica sujeira ou resíduos. |
| RF06      | O robô deve enviar os dados referentes à limpeza de cada tubo para uma base de dados. | User story 3 do Jairo Santos. | O robô envia os dados referentes à limpeza de cada tubo para uma base de dados. | Realizar a limpeza de um tubo. Verificar se o robô envia os dados corretamente para a base de dados. |
