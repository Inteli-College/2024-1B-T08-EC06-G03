---
title: Requisitos Funcionais
---

## Revisão e atualização dos requisitos

Durante a primeira sprint, foram mapeados [requisitos funcionais](Sprint-1/Arquitetura-de-Solucao/requisitos-funcionais.md), que são basicamente as funcionalidades essenciais do projeto. Porém, no decorrer do tempo, o entendimento sobre o projeto em questão mudou, alterando também o rumo do desenvolvimento.
Dessa forma, aqui nessa seção, serão detalhados os requisitos funcionais com todas as alterações realizadas.

| Requisito | Descrição | [User story relacionada](Sprint-1/Design/personas.md) | Critérios de aceitação | Caso de teste |
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

- **Hardware:** Realizar a inicialização do robô, utilizando o método de conexão por ssh para rodar o bringup, que inicializa vários protocolos para o funcionamento do Turtlebot. Para isso, é só abrir um terminal no computador e rodar o comando `ssh bobolins@10.128.0.24`, e digitar a senha para finalizar a conexão. Após isso, é só rodar o comando de bringup, que é `ros2 launch turtlebot3_bringup robot.launch.py`. Após isso, posicionar o robô em um ambiente seguro para a testagem, como uma pista.

- **Software:** Inicializar o Backend, que é composto por serviços ROS, a API da aplicação web e o modelo de visão computacional que irá avaliar as fotos dos tubos.

- **Interface de Controle:** Deixar a interface de controle aberta num dispositivo móvel que será entregue posteriormente ao testador.

### Público alvo dos testadores

Por não ser possível testar com os funcionários que utilizarão o sistema na Atvos, foi selecionado um grupo de 4 pessoas com um **letramento digital básico**, com uma **média de idades de 27 anos** e uma **familiarização média/baixa com robótica.**


## Realização dos testes

Aqui serão explicados os cenários de teste e como será a realização deles. Cada um dos tópicos abaixo indica uma tarefa que o testador irá ter que cumprir.

### Tarefas do usuário durante o teste

#### Teste de Teleoperação (RF01):

Acesse a interface de controle e verifique a transmissão de imagens em tempo real.
Envie comandos ao robô e observe se a mudança das imagens acompanha a movimentação do robô.

#### Teste de Visualização de Dados (RF02):

Visualize a tela de dados e tente achar a informação de quantos canos sujos foram detectados na última inspeção realizada.

#### Teste de Cancelamento de Inspeção (RF03):

Inicie uma inspeção e, após alguns segundos, pressione o botão "kill". Veja se o robô parou imediatamente o seu funcionamento.

#### Teste de Proximidade (RF04):

Aproximar o robô de diferentes obstáculos e observe o sistema de aviso na interface.
Registre se os alertas foram acionados adequadamente e se houve alguma falha no sistema de prevenção de colisão.

#### Teste de Processamento de Imagens (RF05):

Capture imagens de tubos em diferentes condições (limpos, sujos, com resíduos).
Verifique se o sistema identifica corretamente a sujeira ou os resíduos e como ele reporta esses dados.

#### Teste de Envio de Dados (RF06):

Verificar se os dados que constam na página de dados condizem com a realidade: data, número de canos sujos, entre outros.

### Função do avaliador

**Documentação:** Registre todos os resultados dos testes, incluindo falhas encontradas, desempenho do sistema e feedback dos testadores.

**Captura de Evidências:** Caso o testador autorize, grave os testes para documentar o comportamento do sistema e das interfaces.


**Avaliação de Conformidade:** Compare os resultados obtidos com os critérios de aceitação definidos para cada requisito funcional.

**Identificação de Problemas:** Anote quaisquer discrepâncias ou problemas encontrados durante os testes e categorize-os por prioridade.

**Coleta de feedbacks:** Após a finalização do teste, pergunte ao testador como foi a experiência, e se ele tem pontos de melhoria que gostaria de compartilhar.

## Conclusões

Os testes foram realizados no dia 10/06/24, com 4 pessoas diferentes. A interface web foi apresentada num notebook, isto porque a transmissão de imagens no celular estava muito lenta, o que prejudicava a teleoperação. Alguns insigths que foram retirados dessa experiência foram:

- A sensibilidade do controle dificulta um pouco a movimentação do robô;
- Como o teste foi realizado em computador, o controle por joystick não foi muito intuitivo para os usuários, sendo um feedback comum que o controle pelas setas do teclado seria mais fácil;
- Troubleshooting: Foram localizados alguns erros de conexão com o frontend na parte de detecção dos obstáculos;
- Enquanto o teste foi realizado, as partes de visualização de dados e de visão computacional ainda não estavam integradas ao sistema, então **os requisitos relacionados a essas funções (RF02, RF05 e RF06) não puderam ser validados no teste.**

A seguir, serão repassados cada teste e como foi o desempenho deles, problemas encontrados e pontos positivos.

#### Teste de Teleoperação (RF01):

Foi um sucesso em 100% das sessões de teste. A transmissão de imagem em tempo real foi útil e suficiente para a localização do robô no espaço. Porém, em alguns raros momentos, foram registrados alguns travamentos, que foram facilmente resolvidos com uma atualização da página.

#### Teste de Cancelamento de Inspeção (RF03):

Foi um sucesso em 75% das sessões de teste e o robô parou após o botão ser pressionado, não sendo possível movimentá-lo. Porém, na vez em que o comando não funcionou, a interface travou fazendo com que o robô ficasse rodando incessantemente e o controle do mesmo fosse perdido. Como a interface travou, apertar o botão de *kill* não paralizou o robô, o que chamou a atenção para a eficácia desse mecanismo em condições reais de emergência. 

#### Teste de Proximidade (RF04):

Apesar de ser uma funcionalidade implementada desde a [terceira Sprint](https://inteli-college.github.io/2024-1B-T08-EC06-G03/category/sprint-3), houve erros de conexão com o frontend durante os testes, fazendo com que o robô detectasse a presença de obstáculos e bloquasse a movimentação do robô nas direções que o fariam colidir, mas não retornando na interface (para o usuário) os alertas informando a proximidade de um obstáculo. Dessa forma, apesar do impedimento de colisões ter funcionado 100% das vezes, o retorno de alertas no frontend não foi apresentado em nenhum dos testes.


Em conclusão, os testes foram de suma importância para validar algumas das funcionalidades e observar problemas de usabilidade e integração do sistema criado. Os feedbacks coletados irão impactar o desenvolvimento da Sprint a seguir, visando corrigir os problemas encontrados, além de incluir a realização dos testes que se relacionam com os requisitos de funcionamento da detecção de sujidade nos tubos (visão computacional) e visualização de dados.


