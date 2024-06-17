---
title: Requisitos  Não Funcionais
---

## Revisão e atualização dos requisitos

Durante a primeira sprint, foram mapeados os [requisitos não funcionais](/Sprint-1/Arquitetura-de-Solucao/requisitos-nao-funcionais.md) a fim de metrificar e avaliar a qualidade do sistema. Porém, ao longo das sprints, o entendimento sobre o projeto mudou juntamente com o rumo do desenvolvimento. Dessa forma, os requisitos não funcionais foram revisitados e percebeu-se a necessidade de atualizá-los. As mudanças mais substanciais foram referentes à movimentação do robô e aos casos de uso. Assim, a forma de controle é a teleoperação em vez da navegação automática, sendo o robô controlado por um operador  orientado pelas imagens da câmera. Dessa forma, os requisitos relacionados à transmissão de imagem, precisão de movimentação e análise dos tubos também foram alterados, de forma que a transmissão da câmera deve ocorrer em tempo real e não intermitentemente. Além disso, os requisitos de precisão deixaram de ser sobre a movimentação autônoma do robô e passaram a ser sobre a capacidade do teleoperador de posicionar o robô abaixo do tubo. Por fim, outras questões analisadas que não foram consideradas inicialmente foram as características de performance, como o tempo de duração da bateria, capacidade de processamento da visão computacional e a usabilidade, como facilidade do usuário em navegar pela plataforma. A seguir encontra-se a tabela de requisitos não funcionais atualizada. 


| Requisito Não Funcional | Descrição | Teste | Critério de aceitação |
| ----------------------- | --------- | ----- | --------------------- |
| RNF01 | ~~O sistema deve garantir que os dados de status do funcionamento e as imagens coletadas pelo robô sejam transmitidos para o backend através do ROS (Robot Operating System). O intervalo entre cada envio de dados deve ser estritamente de 5 minutos, com uma tolerância de ±1 minuto.~~ | ~~Durante um período de 15 minutos, monitore os intervalos entre cada transmissão de dados e anote o tempo exato de cada transmissão. Em seguida, calcule o intervalo real entre cada transmissão, subtraindo o tempo de transmissão atual do tempo de transmissão anterior. Por fim, verifique se o intervalo real entre cada transmissão de dados está na faixa especificada de 4 a 6 minutos.~~ | ~~1. Os dados de temperatura devem ser enviados do robô para o backend através do ROS em intervalos regulares de 5 minutos. <br/> 2. A tolerância para o intervalo entre os envios de dados é de ±1 minuto, o que significa que o intervalo real pode variar entre 4 e 6 minutos.~~ | 
| RNF02 | O sistema do robô teleoperado deve ser capaz de integrar-se de forma eficiente e interoperável com outros sistemas, permitindo a troca de dados e comandos de maneira transparente e sem problemas. | Serão realizadas solicitações de integração por meio da API do sistema do robô teleoperado, simulando diferentes cenários de uso e tipos de dados. Será monitorada a resposta da API, registrando o tempo de resposta e a taxa de sucesso de cada solicitação. | O teste será considerado bem-sucedido se o tempo médio de resposta da API for inferior a 100 milissegundos e a taxa de sucesso das integrações for superior a 90%. Além disso, a documentação da interface de integração deve ser completa e precisa, facilitando a integração por parte dos desenvolvedores externos. |
| RNF03 |~~O robô deve ser capaz de posicionar com precisão abaixo do tubo para que a câmera esteja alinhada em relação ao centro do tubo. Isso é crucial para garantir uma inspeção eficaz e precisa da superfície interna do tubo.~~ O robô deve ser capaz de se mover com precisão em relação aos comandos enviados pelo controle do operador. Ou seja, a diferença entre o comando enviado pelo joystick da aplicação e o movimento real do robô não deve ultrapassar de ±10°. | ~~1. O sistema de vistoria dos tubos é acionado e o robô se posiciona abaixo do tubo alvo. Além disso, a câmera deve apontar para o interior do alvo. A distância radial entre a câmera e o centro do tubo é medida e comparada com a distância esperada. <br/> 2. Durante a vistoria de limpeza, o robô é deliberadamente desviado de sua posição correta, e o sistema de controle do robô é avaliado em sua capacidade de corrigir o posicionamento da câmera para dentro do limite permitido.~~ <br/>  1. O comando de andar para frente é enviado e o robô anda para a frente. <br/> 2. O comando de virar à esquerda é enviado e o robô gira em seu próprio eixo para a esquerda. <br/> 3. O comando de virar à direita é enviado e o robô gira em seu próprio eixo para a direita. <br/> 4. O comando de andar para trás é enviado e o robô anda para trás. <br/> 5. O operador envia diversos comandos e o robô corresponde a essa mudança. | ~~1. O robô deve se posicionar abaixo do tubo para que a câmera esteja posicionada a no máximo 0.5cm de distância em relação ao centro do tubo. O sistema de posicionamento do robô deve ser capaz de fornecer feedback preciso sobre a posição real do robô durante o processo de inspeção, através das medições dos sensores extras. <br/> 2. O sistema de controle do robô deve ser capaz de perceber caso haja diferenças no posicionamento acima do limite tolerável e se ajustar para garantir a melhor imagem do interior do tubo.~~ A diferença entre o comando enviado pelo operador e o movimento real do robô não deve ultrapassar de ±10°. |
| RNF04 | O robô deve ser capaz de estabelecer uma conexão de alta taxa de transferência com o sistema de armazenamento e processamento das imagens,enviando as imagens tiradas pela câmera no máximo a cada 100 milissegundos, com uma tolerância ±50 milissegundos, garantindo uma entrega rápida das informações para análise (entre 50 e 150 milissegundos). | Verificar o recebimento de informações no ~~backend onde será realizado o processamento das imagens para verificar a periodicidade das informações~~,  frontend, passando pelo backend calculando o tempo entre o recebimento de uma mensagem e outra. | A periodicidade de envio das imagens capturadas deve ser em intervalos regulares de 100 milissegundos, podendo variar entre 50 e 150, considerando a tolerência de ±50 milissegundos. |
| RNF05 | O robô deve apresentar uma precisão mínima na identificação de reboilers que necessitam de limpeza. Isso é essencial para garantir que todos os reboilers sujos sejam devidamente identificados e limpos. | Preparar um ambiente controlado que simula as condições dos reboilers após a limpeza, incluindo variados níveis de acumulação de resíduos; preparar um conjunto de reboilers com diferentes condições de limpeza, alguns limpos e outros intencionalmente sujos, com resíduos representativos dos encontrados na produção real; permitir que o robô inspecione cada reboiler e classifique se necessita ou não de limpeza; comparar as decisões do robô com as condições pré-estabelecidas para cada reboiler, consideradas como "verdadeiros" para fins de teste; calcular a precisão do robô dividindo o número de identificações corretas pelo número total de inspeções realizadas e analisar os casos de erro para entender as limitações do sistema de visão computacional e sugerir melhorias. | O robô deve alcançar uma precisão superior ou igual a ~~95%~~ 80% na identificação correta de reboilers que necessitam de limpeza. |
| RNF06 |  O robô deve ser capaz de se movimentar por um período mínimo de 1 hora sem recarga. | O robô deve ser carregado completamente e deve ser monitorado o tempo de duração da bateria. |  O robô deve ser capaz de se movimentar por um período mínimo de 1 hora sem recarga.|
| RNF07 |  A interface deve ser intuitiva e de fácil utilização. | Aplicação do teste SUS [(System Usability Scale)](https://brasil.uxdesign.cc/o-que-%C3%A9-o-sus-system-usability-scale-e-como-us%C3%A1-lo-em-seu-site-6d63224481c8) com no mínimo 5 usuários| No mínimo, 60% dos usuários devem atingir o score nível B (notas entre 70-80), e todos devem atingir no mínimo o score nivel C (notas entre 60-70).|

## Realização dos testes

A partir da revisão dos requisitos não funcionais, os testes e critérios de aceitação foram atualizados conforme a tabela acima. A seguir, serão detalhados os testes realizados para cada requisito não funcional:

1. RNF02: Teste de integração do sistema

Para testar a integração do sistema, foi necessário realizar solicitações de integração por meio da API do sistema do robô teleoperado, simulando diferentes cenários de uso e tipos de dados. Foi monitorada a resposta da API, registrando o tempo de resposta e a taxa de sucesso de cada solicitação. Para isso, foi utilizado o Postman para realizar as solicitações e monitorar o tempo de resposta e a taxa de sucesso. Foram realizadas 10 solicitações de integração, simulando diferentes cenários de uso e tipos de dados, e foi verificado se o tempo médio de resposta da API foi inferior a 100 milissegundos e a taxa de sucesso das integrações foi superior a 90%. Além disso, foi verificado se a documentação da interface de integração estava completa e precisa, facilitando a integração por parte dos desenvolvedores externos. O teste foi considerado bem-sucedido se o tempo médio de resposta da API foi inferior a 100 milissegundos e a taxa de sucesso das integrações foi superior a 90%.

3. RNF03: Teste de precisão de movimentação do robô.

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

3. RNF04: Teste de taxa de transferência de imagens

Para testar a transmissão de imagens em tempo real, foi necessário transmitir imagens da câmera do robô para o frontend, passando pelo backend, e verificar se a média do intervalo entre cada transmissão de dados estava entre 50ms e 150ms. Para isso, foi necessário monitorar os intervalos entre cada transmissão de dados e anotar o tempo. Para isso, foi utilizada a exibição do FPS (frames per second) no frontend em um período de 5 minutos e o robô foi exposto a diferentes cenários:
    - Com iluminação adequada;
    - Com pouca iluminação;
    - Com obstáculos parados;
    - Com obstáculos se movimentando;
    - Com o acionamento do processamento do modelo de visão computacional.

Após a observação dos diferentes cenários, foi analisado se alguns dos fatores destacados aumentaram a latência da transmissão ou se algum outro fator interferiu na transmissão das imagens. O teste foi considerado bem-sucedido se a média do intervalo entre cada transmissão de dados estava entre 50ms e 150ms.

4. RNF05: Teste de precisão na identificação de reboilers.

Para esse teste, foi necessário simular um ambiente controlado que simula as condições dos reboilers após a limpeza, incluindo variados níveis de acumulação de resíduos. Para isso, foram utilizados os cestos de lixo da sala, que foram preenchidos com diferentes tipos de resíduos, como papel. O teste visava avaliar a precisão do modelo de visão computacional em diferentes contextos e, dado que a simulação mais fidedigna no campus do inteli foram os cestos de lixo, foi decidido que o teste seria realizado com eles. O teste foi considerado bem-sucedido se o robô alcançou uma precisão superior ou igual a 80% na identificação correta de tubos que necessitavam de limpeza, ou seja, tinham algum resíduo. Esse cálculo deve ser feito a partir da divisão do número de identificações corretas pelo número total de tubos que foram indicados como sujos, ou seja, a precisão do modelo de visão computacional.

5. RNF06: Teste de duração da bateria

Nesse teste, a bateria do robô foi carregada completamente e foi monitorado o tempo de duração da bateria. O teste foi considerado bem-sucedido se a bateria permitiu o robô de se movimentar por um período mínimo de 1 hora sem descarregar a bateria.

6. RNF07: Teste de usabilidade

Para testar a usabilidade da interface, foi aplicado o teste SUS (System Usability Scale) com no mínimo 5 usuários. O teste foi considerado bem-sucedido se no mínimo 60% dos usuários atingiram o score nível B (notas entre 70-80), e todos atingiram no mínimo o score nível C (notas entre 60-70). Apesar do público alvo ser engenheiros de manutenção e trabalhadores da equipe de manutenção da Atvos, não foi possível realizar o teste com esse público, devido a restrições de contato com esse público. Dessa forma, o teste foi realizado com alunos e colaboradores do Inteli que, apesar de não serem o público alvo, possuem letramento digital semelhante e podem dar feedbacks relevantes sobre a usabilidade da interface. O teste foi realizado com o teste da interface de controle sem informações prévias, apenas com uma visão geral do escopo do projeto apresentado aos usuários e uma explicação breve sobre as funcionalidades existentes do robô. Com isso, foi possível testar a intuitividade da interface e a facilidade de uso da aplicação no geral.

## Resultados

1. Para o teste de integração do sistema, o tempo médio de resposta da API foi de 50ms e a taxa de sucesso das integrações foi de 100%, o que indica que o teste foi bem-sucedido.Vale ressaltar que a documentação foi simples e intuitiva para outros membros da equipe que não ficaram responsáveis pelo desenvolvimento da API, o que reduziu os vieses e comprovou a clareza da documentação. 

2. Para o teste de precisão de movimentação do robô, a diferença entre o comando enviado pelo operador e o movimento real do robô não ultrapassou de ±10° em nenhum dos casos, a partir da observação da equipe, o que foi considerado um resultado sasisfatório.

3. No teste de taxa de transferência de imagens, a média do intervalo entre cada transmissão de dados foi de 103ms, o que indica que o teste foi bem-sucedido. Nos diferentes cenários, vale destacar observações importantes:

    - Não houve diferença entre a transmissão de imagens com iluminação adequada e com pouca iluminação, o que indica que a iluminação não impactou na latência da transmissão;
    - Não houve diferenças significativas entre obstáculos parados ou em movimento;
    - Não foi possível avaliar o impacto do acionamento do processamento do modelo de visão computacional dado que durante os testes a integração entre o modelo e a aplicação não estava completa.

    Outro ponto a se considerar que não foi previsto no roteiro de testes foi a conexão da rede, que impacta diretamente na transmissão, ou seja, é um ponto essencial a ser avaliado quando o protótipo for testado em campo ou atualizado para a versão de mercado.

4. Para o quinto teste, de duração da bateria,o robô permaneceu ligado continuamente com os pacotes ROS de movimentação, lidar e câmera rodando por 2horas e 25 minutos, o que surpreendeu a equipe. Porém, vale considerar testes futuros com o robô em movimento e com o modelo de visão computacional acionado, para avaliar o consumo de bateria em situações mais próximas do cenário real.

6. Para o teste de usabilidade, 4 usuários testaram a solução com o teste SUS e a média de notas foi de 84,4, indicando um score médio A, uma nota muito boa. Todos os usuários alcançaram a nota B, superando as metas estabelecidas para esse teste. As principais queixas dos usuários foram em relação à qualidade da câmera, à instabilidade de conexão e falta de movimentação pelas setas , pois o teste foi realizado pelo computador, enquanto a aplicação foi desenvolvida para ser utilizada em um dispositivo móvel. Porém, algo muito positivo foi a clareza dos botões e simplicidade da interface. Para ver mais sobre cada teste, acesse a [Tabela de Testes](https://docs.google.com/spreadsheets/d/1hyU2EtLaos3EX-l6XtDDiVFYKS4Vnd7YyudwoYwAVCQ/edit?usp=sharing).

## Conclusões

A partir dos testes realizados, a equipe refletiu sobre os feedbacks e a principal ação a ser tomada na próxima sprint será em relação à melhoria do modelo de visão computacional, adicionar de alguma forma a informação sobre o botão de kill e quando acioná-lo, além de um feedback depois que ele for acionado. Além disso, também será considerado melhorar a sensibilidade do joystick e o feedback de obstáculos pŕoximos. Portanto, os testes de requisitos não funcionais foram fundamentais para verificar pontos de melhoria no sistema e atualizar as necessidades e requisitos do projeto.

## Demonstração

Segue abaixo um vídeo gravado durante um dos procedimentos de teste dos Requisitos Funcionais:

[Teste de Requisitos não funcionais](https://youtu.be/OGPyP2dI_So)

