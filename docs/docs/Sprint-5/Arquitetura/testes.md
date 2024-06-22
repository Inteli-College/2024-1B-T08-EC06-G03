---
title: Testes
---

## Validação dos requisitos funcionais

### Pontos principais a serem testados

- Interface e controle de Teleoperação;
- Transmissão de imagem em tempo real;
- Visualização de dados (funcionamento da conexão com o banco de dados).

#### Preparação para os testes

- **Hardware:** Realizar a inicialização do robô, utilizando o método de conexão por ssh para rodar o bringup, que inicializa vários protocolos para o funcionamento do Turtlebot. Para isso, é só abrir um terminal no computador e rodar o comando `ssh rebolins@rebolins.local`, e digitar a senha para finalizar a conexão (lembre-se de que seu computador e o robô devem estar conectados na mesma rede). Após isso, é só rodar o comando de bringup, que é `ros2 launch turtlebot3_bringup robot.launch.py`. Após isso, posicionar o robô em um ambiente seguro para a testagem, como uma pista.

- **Software:** Inicializar o Backend, composto por serviços ROS, a API da aplicação web e o modelo de visão computacional que irá avaliar as fotos dos tubos.

- **Interface de Controle:** Deixar a interface de controle aberta num dispositivo móvel que será entregue posteriormente ao testador.

#### Público alvo dos testadores

Por não ser possível testar com os funcionários que utilizarão o sistema na Atvos, foi selecionado um grupo de 4 pessoas com um **letramento digital básico**, com uma **média de idades de 27 anos** e uma **familiarização média/baixa com robótica.**

### Tarefas do usuário durante o teste

Aqui serão explicados os cenários de teste e como será a realização deles. Cada um dos tópicos abaixo indica uma tarefa que o testador irá ter que cumprir.


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


# Validação dos requisitos não funcionais

#### RNF01: Teste de integração do sistema

Para testar a integração do sistema, foi necessário realizar solicitações de integração por meio da API do sistema do robô teleoperado, simulando diferentes cenários de uso e tipos de dados. Foi monitorada a resposta da API, registrando o tempo de resposta e a taxa de sucesso de cada solicitação. Para isso, foi utilizado o Postman para realizar as solicitações e monitorar o tempo de resposta e a taxa de sucesso. Foram realizadas 10 solicitações de integração, simulando diferentes cenários de uso e tipos de dados, e foi verificado se o tempo médio de resposta da API foi inferior a 100 milissegundos e a taxa de sucesso das integrações foi superior a 90%. Além disso, foi verificado se a documentação da interface de integração estava completa e precisa, facilitando a integração por parte dos desenvolvedores externos. O teste foi considerado bem-sucedido se o tempo médio de resposta da API foi inferior a 100 milissegundos e a taxa de sucesso das integrações foi superior a 90%.

#### RNF02: Teste de taxa de transferência de imagens

Para testar a transmissão de imagens em tempo real, foi necessário transmitir imagens da câmera do robô para o frontend, passando pelo backend, e verificar se a média do intervalo entre cada transmissão de dados estava entre 50ms e 150ms. Para isso, foi necessário monitorar os intervalos entre cada transmissão de dados e anotar o tempo. Para isso, foi utilizada a exibição do FPS (frames per second) no frontend em um período de 5 minutos e o robô foi exposto a diferentes cenários:
    - Com iluminação adequada;
    - Com pouca iluminação;
    - Com obstáculos parados;
    - Com obstáculos se movimentando;
    - Com o acionamento do processamento do modelo de visão computacional.

Após a observação dos diferentes cenários, foi analisado se alguns dos fatores destacados aumentaram a latência da transmissão ou se algum outro fator interferiu na transmissão das imagens. O teste foi considerado bem-sucedido se a média do intervalo entre cada transmissão de dados estava entre 50ms e 150ms.

#### RNF03 e RNF04: Teste de precisão de movimentação do robô.

Para testar a precisão de movimentação do robô, foi necessário enviar comandos de movimentação para o robô e verificar se a diferença entre o comando enviado pelo operador e o movimento real do robô não ultrapassou de ±10°. Para isso, foi utilizado um joystick para enviar comandos de movimentação para o robô e foi verificado se a diferença entre o comando enviado pelo operador e o movimento real do robô não ultrapassou de ±10°. Foram enviados os seguintes comandos de movimentação para o robô:
    - Comando de andar para frente;
    - Comando de virar à esquerda;
    - Comando de virar à direita;
    - Comando de andar para trás;
    - Comando de andar para frente e em seguida de virar à esquerda sem soltar o joystick;
    - Comando de andar para frente e em seguida de virar à direita sem soltar o joystick;
    - Comando de giro 360° para a esquerda;
    - Comando de giro 360° para a direita.

Durante cada comando, foi verificada a diferença entre o comando enviado e a movimentação real. O teste foi considerado bem-sucedido se a diferença entre o comando enviado pelo operador e o movimento real do robô não ultrapassou de ±10°.

#### RNF05: Teste de precisão na identificação de reboilers.

Para esse teste, foi necessário simular um ambiente controlado que simula as condições dos reboilers após a limpeza, incluindo variados níveis de acumulação de resíduos. Para isso, foram utilizados os cestos de lixo da sala, que foram preenchidos com diferentes tipos de resíduos, como papel. O teste visava avaliar a precisão do modelo de visão computacional em diferentes contextos e, dado que a simulação mais fidedigna no campus do inteli foram os cestos de lixo, foi decidido que o teste seria realizado com eles. O teste foi considerado bem-sucedido se o robô alcançou uma precisão superior ou igual a 80% na identificação correta de tubos que necessitavam de limpeza, ou seja, tinham algum resíduo. Esse cálculo deve ser feito a partir da divisão do número de identificações corretas pelo número total de tubos indicados como sujos, ou seja, a precisão do modelo de visão computacional.

#### RNF06: Teste de duração da bateria

Nesse teste, a bateria do robô foi carregada completamente e teve seu tempo de duração monitorado. O teste foi considerado bem-sucedido se a bateria permitiu o robô de se movimentar por um período mínimo de 1 hora sem descarregar a bateria.

#### RNF07: Teste de usabilidade

Para testar a usabilidade da interface, foi aplicado o teste SUS (System Usability Scale) com no mínimo 5 usuários. O teste foi considerado bem-sucedido se no mínimo 60% dos usuários atingiram o score nível B (notas entre 70-80), e todos atingiram no mínimo o score nível C (notas entre 60-70). Apesar do público alvo ser engenheiros de manutenção e trabalhadores da equipe de manutenção da Atvos, não foi possível realizar o teste com esse público, devido a restrições de contato com esse público. Dessa forma, o teste foi realizado com alunos e colaboradores do Inteli que, apesar de não serem o público alvo, possuem letramento digital semelhante e podem dar feedbacks relevantes sobre a usabilidade da interface. O teste foi realizado com o teste da interface de controle sem informações prévias, apenas com uma visão geral do escopo do projeto apresentado aos usuários e uma explicação breve sobre as funcionalidades existentes do robô. Com isso, foi possível testar a intuitividade da interface e a facilidade de uso da aplicação no geral.

## Resultados

### Validação dos requisitos funcionais

Os testes foram realizados no dia 10/06/24, com 4 pessoas diferentes. A interface web foi apresentada num notebook, isto porque a transmissão de imagens no celular estava muito lenta, o que prejudicava a teleoperação. Alguns insights que foram retirados dessa experiência foram:

- A sensibilidade do controle dificulta um pouco a movimentação do robô;
- Como o teste foi realizado em computador, o controle por joystick não foi muito intuitivo para os usuários, sendo um feedback comum que o controle pelas setas do teclado seria mais fácil;
- Troubleshooting: Foram localizados alguns erros de conexão com o frontend na parte de detecção dos obstáculos;
- Enquanto o teste foi realizado, as partes de visualização de dados e de visão computacional ainda não estavam integradas ao sistema, então **os requisitos relacionados a essas funções (RF02, RF05 e RF06) não puderam ser validados no teste.**

<div align="center">

**Observações dos testes dos requisitos funcionais**

![Validações RFs](/img/testes-rf.png)

Fonte: Elaborado pela equipe Rebólins

</div>

A seguir, serão repassados cada teste e como foi o desempenho deles, problemas encontrados e pontos positivos.

#### Teste de Teleoperação (RF01):

Foi um sucesso em 100% das sessões de teste. A transmissão de imagem em tempo real foi útil e suficiente para a localização do robô no espaço. Porém, em alguns raros momentos, foram registrados alguns travamentos, que foram facilmente resolvidos com uma atualização da página.

#### Teste de Cancelamento de Inspeção (RF03):

Foi um sucesso em 75% das sessões de teste e o robô parou após o botão ser pressionado, não sendo possível movimentá-lo. Porém, na vez em que o comando não funcionou, a interface travou, fazendo com que o robô ficasse rodando incessantemente e o controle do mesmo fosse perdido. Como a interface travou, apertar o botão de *kill* não paralizou o robô, o que chamou a atenção para a eficácia desse mecanismo em condições reais de emergência. 

#### Teste de Proximidade (RF04):

Apesar de ser uma funcionalidade implementada desde a [terceira Sprint](https://inteli-college.github.io/2024-1B-T08-EC06-G03/category/sprint-3), houve erros de conexão com o frontend durante os testes, fazendo com que o robô detectasse a presença de obstáculos e bloqueasse a movimentação do robô nas direções que o fariam colidir, mas não retornando na interface (para o usuário) os alertas informando a proximidade de um obstáculo. Dessa forma, apesar do impedimento de colisões ter funcionado 100% das vezes, o retorno de alertas no frontend não foi apresentado em nenhum dos testes.


### Validação dos requisitos não funcionais


#### Teste de Integração do sistema (RNF01):

Para o teste de integração do sistema, o tempo médio de resposta da API foi de 50ms e a taxa de sucesso das integrações foi de 100%, o que indica que o teste foi bem-sucedido.Vale ressaltar que a documentação foi simples e intuitiva para outros membros da equipe que não ficaram responsáveis pelo desenvolvimento da API, o que reduziu os vieses e comprovou a clareza da documentação. 

#### Teste de Integração do sistema (RNF02):

No teste de taxa de transferência de imagens, a média do intervalo entre cada transmissão de dados foi de 103ms, o que indica que o teste foi bem-sucedido. Nos diferentes cenários, vale destacar observações importantes:

- Não houve diferença entre a transmissão de imagens com iluminação adequada e com pouca iluminação, o que indica que a iluminação não impactou na latência da transmissão;
- Não houve diferenças significativas entre obstáculos parados ou em movimento;
- Não foi possível avaliar o impacto do acionamento do processamento do modelo de visão computacional dado que durante os testes a integração entre o modelo e a aplicação não estava completa.

    Outro ponto a se considerar que não foi previsto no roteiro de testes foi a conexão da rede, que impacta diretamente na transmissão, ou seja, é um ponto essencial a ser avaliado quando o protótipo for testado em campo ou atualizado para a versão de mercado.

#### Teste de precisão da movimentação do robô (impacta no RNF03 e RNF04):

Para o teste de precisão de movimentação do robô, a diferença entre o comando enviado pelo operador e o movimento real do robô não ultrapassou de ±10° em nenhum dos casos, a partir da observação da equipe, o que foi considerado um resultado satisfatório.


#### Teste de duração da bateria (RNF04):

Para o quinto teste, de duração da bateria, o robô permaneceu ligado continuamente com os pacotes ROS de movimentação, lidar e câmera rodando por 2horas e 25 minutos, o que surpreendeu a equipe. Porém, vale considerar testes futuros com o robô em movimento e com o modelo de visão computacional acionado, para avaliar o consumo de bateria em situações mais próximas do cenário real.

#### Teste de Usabilidade (RNF05):

<div align="center">

**Teste de usabilidade SUS**

![Teste SUS](/img/sus.png)

Fonte: Elaborado pela equipe Rebólins

</div>

Para o teste de usabilidade, 4 usuários testaram a solução com o teste SUS e a média de notas foi de 84,4, indicando um score médio A, uma nota muito boa. Todos os usuários alcançaram a nota B, superando as metas estabelecidas para esse teste. As principais queixas dos usuários foram em relação à qualidade da câmera, à instabilidade de conexão e falta de movimentação pelas setas, pois o teste foi realizado pelo computador, enquanto a aplicação foi desenvolvida para ser utilizada em um dispositivo móvel. Porém, algo muito positivo foi a clareza dos botões e simplicidade da interface. Para ver mais sobre cada teste, acesse a [Tabela de Testes](https://docs.google.com/spreadsheets/d/1hyU2EtLaos3EX-l6XtDDiVFYKS4Vnd7YyudwoYwAVCQ/edit?usp=sharing).


Segue abaixo um vídeo gravado durante um dos procedimentos de teste dos Requisitos Funcionais:

<div align="center">
<iframe width="560" height="315" src="https://www.youtube.com/embed/OGPyP2dI_So?si=ebAhvx2e7egPQex9" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe> 
</div>