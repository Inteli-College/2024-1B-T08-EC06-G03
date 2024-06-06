---
title: Requisitos Funcionais
---

Durante a primeira sprint, foram mapeados [requisitos funcionais](../../Sprint%201/Arquitetura%20de%20Solucao/Requisitos%20Funcionais.md), que são basicamente as funcionalidades essenciais do projeto. Porém, no decorrer do tempo, o entendimento sobre o projeto em questão mudou, alterando também o rumo do desenvolvimento.
Dessa forma, aqui nessa seção, serão detalhados os requisitos funcionais com todas as alterações realizadas.

| Requisito | Descrição | [User story relacionada](../../Sprint%201/Design/personas.md) | Critérios de aceitação | Caso de teste |
|-----------|-----------|-----------------------|------------------------|---------------|
| RF01      | A interface de usuário permite que o robô seja controlado de maneira teleoperada | User story 1 do Danillo | O sistema apresenta acompanhamento das imagens enviadas pela câmera em tempo real, além de contar com uma interface de controle do robô | Verificação da transmissão de imagem em tempo real e do funcionamento do controle|
| RF02      | É possível acessar dados sobre uma inspeção | User story 2 do Jairo | É possível visualizar numa tabela qual a quantidade de canos sujos em uma inspeção | Visualizar a tela de visualização de dados|
| RF03      | O operador deve ser capaz de cancelar a inspeção a qualquer momento, de modo que o robô pare sua ação em no máximo 10 segundos. | User story 4 do Danillo Chrystian | O robô para sua ação em no máximo 10 segundos quando a inspeção é cancelada. | Pressionar o botão de cancelar durante a inspeção e verificar se o robô para sua ação em no máximo 10 segundos. |
| RF04      | Deve ser possível saber a proximidade do robô em relação às paredes, a fim de ser possível parar o robô antes de uma colisão| User story 3 do Jairo Santos. | A interface possui um aviso indicando a aproximação de um obstáculo | Aproximar o robô de um obstáculo e visualizar o sistema de alarme na tela|
| RF05      | O sistema deve processar as imagens capturadas para identificar sujeira ou resíduos, utilizando algoritmos de visão computacional. | User story 2 do Jairo Santos. | O sistema processa as imagens capturadas para identificar sujeira ou resíduos. | Capturar imagens. Verificar se o sistema identifica sujeira ou resíduos. |
| RF06      | O robô deve enviar os dados referentes à limpeza de cada tubo para uma base de dados. | User story 3 do Jairo Santos. | O robô envia os dados referentes à limpeza de cada tubo para uma base de dados. | Realizar a limpeza de um tubo. Verificar se o robô envia os dados corretamente para a base de dados. |

## Mapeamento dos testes

Com base nos requisitos funcionais, é possível mapear os testes que comprovam o funcionamento do sistema.

### Pontos principais a serem testados

- Interface e controle de Teleoperação;
- Transmissão de imagem em tempo real;
- Visualização de dados (funcionamento da conexão com o banco de dados).

### Preparação para os testes

- **Hardware:** Certifique-se de que o robô esteja completamente funcional e que todos os componentes necessários, como sensores, câmeras e sistemas de controle, estejam instalados e operacionais.

- **Software:** Instale a versão mais recente do software de controle e processamento de imagem no sistema que será utilizado para os testes. Garanta que todos os drivers e dependências estejam atualizados.

- **Base de Dados:** Configure uma base de dados de testes que possa receber os dados enviados pelo robô. Esta base de dados deve ser isolada da produção para evitar qualquer interferência ou perda de dados reais.

- **Interface de Controle:** Configure e verifique a interface de usuário que será utilizada para controlar o robô e monitorar as inspeções.

### Público alvo dos testadores

Por não ser possível testar com os funcionários que utilizarão o sistema na Atvos, foi selecionado um grupo de 4 pessoas com um letramento básico em tecnologia, com idades entre 20 e 23 anos.


## Realização dos testes

Aqui serão explicados os cenários de teste e como será a realização deles. Cada um dos tópicos abaixo indica uma tarefa que o testador irá ter que cumprir.

### Execução dos Cenários de Teste

#### Teste de Teleoperação (RF01):

Acesse a interface de controle e verifique a transmissão de imagens em tempo real.
Envie comandos ao robô e observe a resposta imediata.
Registre qualquer atraso ou falha na transmissão de imagem ou nos comandos.

#### Teste de Visualização de Dados (RF02):

Realize uma inspeção e visualize os dados coletados, incluindo a quantidade de canos sujos.
Verifique a precisão das informações exibidas na tabela.

#### Teste de Cancelamento de Inspeção (RF03):

Inicie uma inspeção e, após alguns segundos, pressione o botão de cancelar.
Meça o tempo que o robô leva para parar suas ações e verifique se está dentro do limite de 10 segundos.

#### Teste de Proximidade (RF04):

Aproximar o robô de diferentes obstáculos e observe o sistema de aviso na interface.
Registre se os alertas foram acionados adequadamente e se houve alguma falha no sistema de prevenção de colisão.

#### Teste de Processamento de Imagens (RF05):

Capture imagens de tubos em diferentes condições (limpos, sujos, com resíduos).
Verifique se o sistema identifica corretamente a sujeira ou os resíduos e como ele reporta esses dados.

#### Teste de Envio de Dados (RF06):

Realize uma operação de limpeza e monitore o envio dos dados para a base de dados.
Verifique a integridade e a consistência dos dados armazenados.
Registro dos Resultados

Documentação: Registre todos os resultados dos testes, incluindo falhas encontradas, desempenho do sistema e feedback dos testadores.

Captura de Evidências: Capture capturas de tela e vídeos durante os testes para documentar o comportamento do sistema e das interfaces.

#### Análise dos Resultados

Avaliação de Conformidade: Compare os resultados obtidos com os critérios de aceitação definidos para cada requisito funcional.
Identificação de Problemas: Anote quaisquer discrepâncias ou problemas encontrados durante os testes e categorize-os por prioridade.

#### Relatório Final

Sumário dos Testes: Compile um relatório detalhado que inclua os resultados dos testes, as falhas identificadas e as sugestões de melhorias.

Recomendações: Forneça recomendações para correções e ajustes necessários antes de o sistema ser implantado em ambiente de produção.

#### Feedback e Melhorias

Sessão de Feedback: Realize uma sessão de feedback com os testadores para entender melhor suas experiências e sugestões.

Planejamento de Correções: Baseado no feedback e nos resultados dos testes, planeje as correções e melhorias necessárias para os próximos ciclos de desenvolvimento.

## Conclusões