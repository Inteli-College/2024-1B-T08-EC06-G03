---
title: Requisitos Funcionais
---

Durante a primeira sprint, foram mapeados [requisitos funcionais](../../Sprint%201/Arquitetura%20de%20Solucao/Requisitos%20Funcionais.md), que são basicamente as funcionalidades essenciais do projeto. Porém, no decorrer do tempo, o entendimento sobre o projeto em questão mudou, alterando também o rumo do desenvolvimento.
Dessa forma, aqui nessa seção, serão detalhados os requisitos funcionais com todas as alterações realizadas.

| Requisito | Descrição | User story relacionada | Critérios de aceitação | Caso de teste |
|-----------|-----------|-----------------------|------------------------|---------------|
| RF01      | A interface de usuário permite que o robô seja controlado de maneira teleoperada | User story 2 do Danillo Chrystian | O sistema apresenta uma interface de usuário simplificada com passos claros e simples para iniciar a execução. | Verificar se a interface de usuário apresenta os passos corretos para iniciar a execução. |
| RF02      | O robô deve ser capaz de se movimentar com base em um algoritmo de movimentação, alimentado por um input da planta do reboiler enviado na interface do usuário. | User story 1 do Danillo Chrystian | O robô se movimenta corretamente com base no algoritmo de movimentação. | Enviar um input da planta do reboiler na interface do usuário e verificar se o robô se movimenta corretamente. |
| RF03      | O operador deve ser capaz de cancelar a inspeção a qualquer momento, de modo que o robô pare sua ação em no máximo 10 segundos. | User story 4 do Danillo Chrystian | O robô para sua ação em no máximo 10 segundos quando a inspeção é cancelada. | Pressionar o botão de cancelar durante a inspeção e verificar se o robô para sua ação em no máximo 10 segundos. |
| RF04      | O robô deve ter um sistema de iluminação que possibilite uma melhor captação da imagem do tubo. | User story 3 do Jairo Santos. | O sistema de iluminação do robô permite uma melhor captação da imagem do tubo. | Verificar se o sistema de iluminação do robô melhora a captação da imagem do tubo. |
| RF05      | O sistema deve processar as imagens capturadas em tempo real para identificar sujeira ou resíduos, utilizando algoritmos de visão computacional. | User story 2 do Jairo Santos. | O sistema processa as imagens capturadas em tempo real para identificar sujeira ou resíduos. | Capturar imagens em tempo real. Verificar se o sistema identifica sujeira ou resíduos. |
| RF06      | O robô deve ser capaz de comunicar o status da inspeção e quaisquer problemas detectados em tempo real ao operador ou ao sistema central. | User story 3 do Jairo Santos. | O robô emite alertas quando identifica problemas em seu funcionamento | Simular falhas em funcionalidades específicas e verificar o comportamento do robô. |
| RF07      | O robô deve enviar os dados referentes à limpeza de cada tubo para uma base de dados central. | User story 3 do Jairo Santos. | O robô envia os dados referentes à limpeza de cada tubo para uma base de dados central. | Realizar a limpeza de um tubo. Verificar se o robô envia os dados corretamente para a base de dados central. |
| RF08      | O robô deve ser capaz de ser controlado remotamente em casos de emergência ou irregularidades da inspeção.  | User story 5 do Danillo Chrystian | O robô pode ser controlado remotamente em casos de emergência ou irregularidades da inspeção. | Simular um caso de emergência ou irregularidade na inspeção e verificar se o robô pode ser controlado remotamente. |

## Mapeamento dos testes



## Realização dos testes


## Conclusões