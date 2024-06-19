# Arquitetura da Informação

A arquitetura da informação é um elemento crucial no design de sistemas interativos, delineando como as informações são organizadas, gerenciadas e disponibilizadas aos usuários. Essencialmente, ela serve como o esqueleto estrutural de um sistema, orientando a criação de interfaces e a interação do usuário de maneira lógica e eficiente. No contexto deste projeto, a arquitetura da informação define como os dados sobre a limpeza de reboilers são captados, processados e apresentados, garantindo que os usuários, como o Engenheiro de Manutenção e o Operador de Máquinas, tenham acesso fácil e intuitivo às informações necessárias para realizar suas tarefas com eficácia.
A arquitetura da informação aqui detalhada inclui diagramas de relação e sequência que mapeiam a interação entre os usuários e o sistema. Esses diagramas ajudam a ilustrar o fluxo de informação entre os componentes do sistema, as operações executadas pelo robô, e como os dados coletados são processados e apresentados.

## Diagrama de Relação

A arquitetura de relação descreve como os componentes do sistema estão interligados e como a informação é compartilhada entre eles:

- **Interface do Usuário:**
  - **Painel de Jairo (Engenheiro de Manutenção):**
    - Visualização de dados históricos e em tempo real sobre a limpeza dos tubos.
    - Alertas de manutenção e relatórios de eficiência.
  - **Painel de Danillo (Operador de Máquinas):**
    - Controles para operar o robô.
    - Status do mapeamento e limpeza.
    - Notificações de tarefas completas ou pendentes.

- **Sistema de Controle do Robô:**
  - Recebe comandos para iniciar, pausar, ou parar inspeções.
  - Envia status operacional para a interface do usuário.

- **Sistema de Monitoramento e Análise de Dados:**
  - Processa dados dos sensores do robô.
  - Gera visualizações sobre a eficiência da limpeza.

- **Base de Dados:**
  - Armazena todos os dados coletados e gerados.
  - Acessível por APIs para consulta e relatórios.

## Diagrama de Sequência

Este diagrama mostra as etapas envolvidas em uma operação típica de inspeção e como o sistema responde a cada ação:

1. **Início da Operação de Inspeção por Danillo:**
   - Seleciona "Iniciar Inspeção" na interface.
   - Sistema verifica condições de operação do robô.
   - Robô inicia a limpeza após verificação.

2. **Execução da Inspeção pelo Robô:**
   - Navega pelos canos usando o mapeamento prévio.
   - Sensores avaliam a limpeza e coletam dados.

3. **Processamento dos Dados:**
   - Dados enviados ao sistema de monitoramento e análise.
   - Determinação de áreas que necessitam de limpeza adicional.

4. **Feedback para Danillo e Jairo:**
   - Danillo recebe notificação sobre necessidade de limpeza adicional.
   - Jairo tem acesso a visualizações de dados acerca da condição de limpeza dos tubos.

Uma visualização do sistema apresentado acima pode ser visto no diagrama abaixo:


<div align="center">

**Arquitetura de Informação**

![Diagrama de sequência](/img/arquitetura-de-informacao.jpg)

**Fonte:** Elaborado pela equipe Rebólins

</div>