---
title: Banco de dados
sidebar_position: 3
---

# Banco de Dados

## Introdução

Na seção [Visualização de Dados](../Sprint%203/Visualização%20de%20Dados.md), foram apresentados possíveis casos de uso dos dados que serão coletados pelo projeto. Nesta seção, haverá o detalhamento como esses dados estão armazenados dentro do **banco de dados relacional**, conforme ilustrado abaixo:

<div align="center">

**Banco de dados Relacional**

![Banco de dados](../../static/img/Banco%20de%20dados.png)


**Fonte:** Elaborado pela equipe Rebólins

</div>

A escolha de um banco de dados relacional foi feita devido à natureza estruturada dos dados do projeto. Essa escolha permite a realização de consultas complexas e filtragens eficientes, facilitadas pela linguagem SQL. Além disso, a integridade e a consistência dos dados são mantidas através das relações entre as tabelas, garantindo um armazenamento robusto e confiável.

## Descrição das tabelas

Abaixo está uma breve descrição de cada tabela e quais informações estão contidas em cada uma.


### Tube

- **id**: Identificador único do tubo. (Chave primária)
- **reboiler_id**: Referência para o reboiler ao qual o tubo está associado.
- **position**: Coordenadas ou posição do tubo.


### Image

- **id**: Identificador único da imagem.
- **taken_at**: Timestamp indicando quando a imagem foi tirada.


### Robot

- **id**: Identificador único do robô. (Chave primária)
- **last_manufactured**: Timestamp indicando a última data de fabricação do robô.

### TubeState

- **id**: Identificador único do estado do tubo. (Chave primária)
- **dirtness**: Booleano indicando se o tubo está sujo.
- **image_id**: Referência para a imagem associada ao estado do tubo.
- **session_id**: Referência para a verificação associada ao estado do tubo.
- **tube_id**: Referência para o tubo associado.

### Examination

- **id**: Identificador único da verificação. (Chave primária)
- **etapa**: Etapa da verificação (enum).
- **robot_id**: Referência para o robô que realizou a verificação.
- **reboiler_id**: Referência para o reboiler que está sendo verificado.
- **started_at**: Timestamp indicando quando a verificação começou.
- **finished_at**: Timestamp indicando quando a verificação terminou.

### Reboiler

- **id**: Identificador único do reboiler. (Chave primária)
- **number**: Número do reboiler.
- **unit_id**: Referência para a unidade à qual o reboiler pertence.

### Unit

- **id**: Identificador único da unidade. (Chave primária)
- **city**: Cidade onde a unidade está localizada.
- **state**: Estado onde a unidade está localizada.

## Relacionamentos

**Tube e Reboiler**

Tube.reboiler_id refere-se a Reboiler.id. Isso cria uma relação muitos-para-um *(muitos tubos podem estar associados a um reboiler).*

**Examination e Robot**

Examination.robot_id refere-se a Robot.id. Isso cria uma relação muitos-para-um *(muitas verificações podem ser realizadas por um robô)*.

**Examination e Reboiler**

Examination.reboiler_id refere-se a Reboiler.id. Isso cria uma relação muitos-para-um *(muitas verificações podem ser associadas a um reboiler)*.

**TubeState e Tube**

TubeState.tube_id refere-se a Tube.id. Isso cria uma relação muitos-para-um *(muitos estados podem ser associados a um tubo)*.

**TubeState e Image**

TubeState.image_id refere-se a Image.id. Isso cria uma relação muitos-para-um *(muitos estados podem ter uma imagem associada)*.

**TubeState e Examination**

TubeState.session_id refere-se a Examination.id. Isso cria uma relação muitos-para-um *(muitos estados podem estar associados a uma verificação)*.

**Reboiler e Unit**

Reboiler.unit_id refere-se a Unit.id. Isso cria uma relação muitos-para-um *(muitos reboilers podem estar associados a uma unidade)*.

## Implementação

Para a implementação do projeto, a equipe optou por utilizar o [Turso](https://turso.tech/), um serviço de banco de dados relacional que oferece diversas vantagens para o desenvolvimento e manutenção do sistema. A escolha do Turso foi justificada pelas seguintes razões:

- **Facilidade de Hospedagem na Máquina Local**: O Turso permite uma configuração simples e eficiente, facilitando a hospedagem do banco de dados diretamente nas máquinas dos desenvolvedores. Isso agiliza o processo de desenvolvimento e testes locais, permitindo uma integração mais suave com o ambiente de desenvolvimento.

- **Disponibilidade Online**: O Turso oferece um serviço online robusto, garantindo que o banco de dados esteja acessível a qualquer momento e de qualquer lugar. Isso é crucial para a colaboração remota da equipe e para o acesso contínuo aos dados durante as diversas etapas do projeto.

- **Custo-Benefício e Escalabilidade**: O Turso é uma solução econômica que se adapta às necessidades do projeto à medida que ele cresce. Sua escalabilidade permite que a equipe expanda a capacidade do banco de dados conforme o aumento do volume de dados e a complexidade das operações, sem incorrer em custos elevados.

- **Interface Simples**: A interface intuitiva do Turso facilita a administração e o gerenciamento do banco de dados. Com uma curva de aprendizado suave, a equipe pode focar no desenvolvimento das funcionalidades do projeto sem se preocupar com a complexidade da gestão do banco de dados.

Esses fatores tornam o Turso a escolha ideal para o projeto, garantindo eficiência, acessibilidade e uma gestão simplificada dos dados, alinhada com as necessidades e objetivos da equipe.





