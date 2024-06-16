---
title: Testes 
sidebar_position: 2
---
# Testes

Estes casos de teste foram desenvolvidos para garantir a qualidade e a conformidade do sistema de inspeção de reboiler. O sistema foi projetado para fornecer uma interface simplificada para iniciar a execução da inspeção, controlar o movimento do robô, identificar problemas durante a inspeção e enviar dados relevantes para uma base de dados central. Além disso, o sistema deve atender às necessidades específicas das personas Jairo e Danillo, conforme detalhado nas user stories relacionadas.

### Caso de Teste 1: Verificar Interface de Usuário para Iniciar a Execução

Requisito Funcional relacionado: RF01
User Story relacionada: User Story 2 do Danillo Chrystian

**Descrição:** Verificar se a interface de usuário apresenta os passos corretos para iniciar a execução de forma simples e intuitiva.

**Passos:**

1. Acessar a interface de usuário do sistema.
2. Identificar a seção ou botão para iniciar a execução.
3. Seguir os passos indicados pela interface para iniciar a execução.

**Verificações:**

- A seção ou botão para iniciar a execução está visível e facilmente acessível.
- Os passos para iniciar a execução estão claramente descritos.
- Não há ambiguidades nos passos indicados pela interface.

### Caso de Teste 2: Verificar Movimentação Precisa do Robô com Base no Input da Planta

Requisito Funcional relacionado: RF02

User Story relacionada: User Story 1 do Danillo Chrystian

**Descrição:** Verificar se o robô se movimenta de forma precisa e eficiente com base no input da planta do reboiler enviado na interface do usuário.

**Passos:**

1. Obter o input da planta do reboiler a ser enviado na interface do usuário.
2. Posicionar o robô próximo ao reboiler para iniciar a execução.
3. Enviar o input da planta do reboiler na interface do usuário.
4. Observar o movimento do robô enquanto executa as instruções baseadas no input da planta.
5. Registrar possíveis desvios ou erros de movimentação do robô.

**Verificações:**

- Verificar se o robô segue precisamente as instruções contidas no input da planta do reboiler.
- Avaliar se o robô executa cada movimento de forma suave e controlada.
- Observar se o robô realiza as paradas necessárias nos pontos indicados pelo input da planta.
- Registrar qualquer desvio, erro ou imprecisão na movimentação do robô e identificar a causa.
- Verificar se o robô retorna à posição inicial de forma precisa após completar a execução do input da planta.

**Comparação (Opcional):**

- Comparar a movimentação do robô com o input da planta enviado na interface do usuário em diferentes condições de operação (por exemplo, com e sem obstáculos no ambiente) para avaliar a consistência do desempenho do robô.
- Analisar a precisão e eficiência do movimento do robô com base no input da planta com diferentes configurações de velocidade e aceleração para otimizar o desempenho do sistema.

### Caso de Teste 3: Verificar Tempo de Parada do Robô ao Cancelar Inspeção

Requisito Funcional relacionado: RF03
User Story relacionada: User Story 4 do Danillo Chrystian

**Descrição:** Verificar se o robô para sua ação em no máximo 10 segundos quando a inspeção é cancelada.

**Passos:**

1. Iniciar a inspeção.
2. Pressionar o botão de cancelar.
3. Observar o tempo que o robô leva para parar sua ação.

**Verificações:**

- O robô para sua ação dentro do prazo de 10 segundos após o cancelamento.
- Não há movimentos residuais do robô após o cancelamento.

### Caso de Teste 4: Verificar Melhoria na Captação de Imagem com Iluminação

Requisito Funcional relacionado: RF04
User Story relacionada: User Story 3 do Jairo Santos

**Descrição:** Verificar se o sistema de iluminação do robô melhora a captação da imagem do tubo em comparação com a ausência de iluminação.

**Passos:**

1. Capturar imagens do tubo sem a iluminação do robô.
2. Registrar as condições de iluminação ambiente durante a captura das imagens.
3. Ativar o sistema de iluminação do robô.
4. Capturar imagens do tubo com a iluminação do robô.
5. Registrar as condições de iluminação fornecidas pelo sistema.

**Verificações:**

- Comparar as imagens capturadas sem iluminação com as imagens capturadas com a iluminação do robô.
- Verificar se a iluminação do robô melhora significativamente a visibilidade da imagem do tubo.
- Observar se há uma redução ou eliminação de áreas escuras ou sombreadas nas imagens com a iluminação do robô.
- Analisar se a qualidade das imagens capturadas com a iluminação do robô permite uma identificação mais clara de detalhes e possíveis sujeiras ou resíduos nos tubos.
- Comparar a nitidez e clareza das imagens capturadas com e sem a iluminação do robô para determinar o impacto positivo da iluminação na captação de imagem.

Com essa comparação entre os resultados obtidos com e sem a iluminação do robô, podemos avaliar de forma mais precisa o impacto da iluminação na captação de imagem e na qualidade dos dados obtidos durante a inspeção.

### Caso de Teste 5: Verificar Identificação de Sujeira ou Resíduos em Imagens

Requisito Funcional relacionado: RF05
User Story relacionada: User Story 2 do Jairo Santos

**Descrição:** Verificar se o sistema identifica sujeira ou resíduos nas imagens capturadas em tempo real.

**Passos:**

1. Capturar imagens em tempo real.
2. Observar as imagens capturadas pelo sistema.

**Verificações:**

- O sistema identifica corretamente sujeira ou resíduos nas imagens.
- Não há falsos positivos ou negativos na identificação.

### Caso de Teste 6: Verificar Emissão de Alertas em Caso de Problemas

Requisito Funcional relacionado: RF06
User Story relacionada: User Story 3 do Jairo Santos

**Descrição:** Verificar se o robô emite alertas de forma adequada quando identifica problemas em seu funcionamento durante a inspeção.

**Passos:**

1. Simular falhas em funcionalidades específicas do robô durante a inspeção.
2. Observar o comportamento do robô após a identificação da falha.
3. Verificar se o robô emite alertas correspondentes às falhas identificadas.
4. Registrar os tipos de alertas emitidos e a sua clareza e precisão.

**Verificações:**

- Verificar se o robô detecta corretamente as falhas em seu funcionamento.
- Observar se o robô emite alertas imediatamente após a identificação da falha.
- Avaliar se os alertas são compreensíveis e indicam claramente a natureza da falha.
- Registrar se os alertas são acompanhados de instruções claras sobre as ações necessárias para resolver a falha.
- Verificar se os alertas são exibidos de forma visível e audível para garantir que sejam prontamente percebidos pelo operador.

**Comparação (Opcional):**

- Comparar a resposta do robô a diferentes tipos de falhas ou situações de erro para avaliar a consistência do sistema de alerta.
- Analisar a eficácia dos alertas emitidos pelo robô em diferentes condições ambientais (por exemplo, com ruído de fundo) para garantir que sejam percebidos pelo operador em todas as situações.

### Caso de Teste 7: Verificar Envio de Dados para Base de Dados Central

Requisito Funcional relacionado: RF07
User Story relacionada: User Story 3 do Jairo Santos

**Descrição:** Verificar se o robô envia corretamente os dados referentes à limpeza de cada tubo para uma base de dados central após a conclusão da inspeção.

**Passos:**

1. Realizar a limpeza de um tubo específico como parte da inspeção.
2. Aguardar a conclusão da limpeza e inspeção por parte do robô.
3. Verificar se os dados referentes à limpeza do tubo são registrados corretamente pelo robô.
4. Verificar se os dados são enviados para a base de dados central.
5. Acessar a base de dados central para confirmar o recebimento dos dados.

**Verificações:**

- Verificar se os dados registrados pelo robô incluem informações completas e precisas sobre a limpeza do tubo (por exemplo, data e hora da limpeza, resultado da inspeção, identificação do tubo).
- Confirmar se os dados são enviados para a base de dados central sem erros ou perdas de informação.
- Verificar se os dados são armazenados corretamente na base de dados central, seguindo o formato e a estrutura definidos.
- Garantir que os dados enviados pelo robô podem ser facilmente acessados e consultados na base de dados central por outros sistemas ou usuários autorizados.
- Confirmar se a base de dados central atualiza corretamente as informações de limpeza do tubo conforme novos dados são recebidos do robô em inspeções subsequentes.

**Comparação (Opcional):**

- Comparar os dados registrados pelo robô com as informações fornecidas pelos operadores para garantir a consistência e integridade dos dados enviados para a base de dados central.
- Analisar o tempo necessário para o envio e processamento dos dados na base de dados central para identificar possíveis gargalos ou atrasos no sistema de comunicação.

### Caso de Teste 8: Verificar Controle Remoto em Casos de Emergência

Requisito Funcional relacionado: RF08
User Story relacionada: User Story 5 do Danillo Chrystian

**Descrição:** Verificar se o robô pode ser controlado remotamente em casos de emergência ou irregularidades da inspeção.

**Passos:**

1. Simular um caso de emergência ou irregularidade durante a inspeção do robô.
2. Acionar o modo de controle remoto a partir da estação de controle.
3. Emitir comandos de controle remoto para o robô.
4. Verificar se o robô responde aos comandos de forma adequada e imediata.
5. Registrar o tempo de resposta do robô aos comandos de controle remoto.

**Verificações:**

- Confirmar se o modo de controle remoto é ativado corretamente em situações de emergência ou irregularidade.
- Verificar se os comandos enviados pela estação de controle são recebidos e interpretados corretamente pelo robô.
- Observar se o robô executa os comandos recebidos de forma precisa e imediata.
- Garantir que o tempo de resposta do robô aos comandos de controle remoto está dentro de limites aceitáveis para situações de emergência.

### Caso de Teste 9: Verificar Informações Fornecidas ao Final da Inspeção

Requisito Funcional relacionado: RF01
User Story relacionada: User Story 2 do Danillo Chrystian

**Descrição:** Verificar se a plataforma informa ao operador se há algum cano sujo ao final da inspeção.

**Passos:**

1. Concluir a inspeção.
2. Verificar as informações fornecidas pela plataforma ao final da inspeção.
3. Identificar se há algum cano considerado sujo conforme os critérios estabelecidos.

**Verificações:**

- Verificar se a plataforma fornece informações claras e precisas sobre o estado de limpeza de cada tubo ao final da inspeção.
- Confirmar se as informações são exibidas de forma visível e compreensível para o operador.
- Garantir que as informações sobre tubos sujos são destacadas de forma adequada para facilitar a identificação e tomada de decisão pelo operador.
- Verificar se as informações fornecidas ao final da inspeção estão de acordo com os critérios estabelecidos para determinar se um tubo está sujo ou limpo.
