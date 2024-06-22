# Arquitetura da Informação

A arquitetura da informação é um elemento crucial no design de sistemas interativos, delineando como as informações são organizadas, gerenciadas e disponibilizadas aos usuários. Essencialmente, ela serve como o esqueleto estrutural de um sistema, orientando a criação de interfaces e a interação do usuário de maneira lógica e eficiente.

No contexto deste projeto, a arquitetura da informação define como os dados sobre a limpeza de reboilers são captados, processados e apresentados, garantindo que os usuários, como o Engenheiro de Manutenção e o Operador de Máquinas, tenham acesso fácil e intuitivo às informações necessárias para realizar suas tarefas com eficácia.

A arquitetura da informação aqui detalhada inclui diagramas de contexto e de sequência que mapeiam a interação entre os usuários e o sistema. Esses diagramas ajudam a ilustrar o fluxo de informação entre os componentes do sistema, as operações executadas pelo robô, e como os dados coletados são processados e apresentados.

## Diagrama de contexto

Um adequado modelo de fluxo de dados de alto nível, o diagrama de contexto ilustra a interação entre um produto e pessoas, organizações ou sistemas externos. É bastante utilizado para entender o contexto e os limites dos sitemas dentro de um projeto.

Os elementos estruturais principais de um diagrama de contexto são o produto, as entidades ou agentes externos e os fluxos de dados. Na nossa solução, foi considerado o produto como um sistema de gerenciamento e teleoperação; os agentes externos como o operador da persona Danillo e o engenheiro de manutenção da persona Jairo; entidades como o robô e o banco de dados e um fluxo de dados como o da imagem que segue.

<div align="center">

**Diagrama de contexto**

![Diagrama de sequência](/img/diagrama_contexto.jpg)

**Fonte:** Elaborado pela equipe Rebólins

</div>

Esse diagrama foi de evidente importância para o projeto. Nos serviu como base para projetar as interfaces do sistema e ajudou na coleta de requisitos. Além disso, foi útil para a comunicação entre as partes interessadas do projeto, oferecendo um entendimento comum do sistema.


## Diagrama de sequência

Os diagramas de sequência são ferramentas essenciais na modelagem da arquitetura da informação, detalhando a ordem dos eventos e a interação entre os diferentes componentes de um sistema ao longo do tempo.

Eles fornecem uma visão clara de como as solicitações são processadas e como as respostas são geradas, facilitando a compreensão do fluxo de informações e a coordenação das ações entre os componentes.

No contexto deste projeto, os diagramas de sequência ajudam a mapear as interações detalhadas entre o operador, o engenheiro de manutenção, o frontend, o backend, o banco de dados e o robô.

Esses diagramas detalham os passos específicos necessários para realizar tarefas como a teleoperação do robô, análise de imagens, e gerenciamento de procedimentos e robôs/reboilers.

### A. Operador Danillo

O diagrama de sequência que segue mostra as interações entre o operador, o frontend, o backend, o banco de dados e o robô durante a sua teleoperação. Ele detalha como os comandos são enviados, processados e executados, e como as informações são retornadas ao operador.

<div align="center">

**Diagrama de sequência - Operador Danillo**

![Diagrama de sequência](/img/diagrama_uml_operador.jpg)

**Fonte:** Elaborado pela equipe Rebólins

</div>

### B. Engenheiro de manutenção Jairo

O diagrama abaixo, por outro lado, mostra as interações entre o engenheiro de manutenção, o frontend, o backend e o banco de dados durante a criação e o gerenciamento de procedimentos e atribuição de robôs/reboilers para cada procedimento. É detalhado como as informações são solicitadas e armazenadas, além de como as interações são coordenadas para garantir a atualização precisa dos dados no sistema.

<div align="center">

**Diagrama de sequência - Engenheiro de manutenção Jairo**

![Diagrama de sequência](/img/diagrama_uml_manutencao.jpg)

**Fonte:** Elaborado pela equipe Rebólins

</div>

Assim como o diagrama de contexto, também os diagramas de sequência foram de inegável importância para o projeto. Eles ajudaram a esclarecer os processos mais internos do sistema ao longo do tempo. Isso influenciou e apoiou o desenvolvimento da solução.

Por fim, semelhantes ao de contexto, esses diagramas de sequência também foram úteis para a comunicação entre as partes interessadas do projeto, facilitando o alinhamento sobre o modus operandi do sistema.


