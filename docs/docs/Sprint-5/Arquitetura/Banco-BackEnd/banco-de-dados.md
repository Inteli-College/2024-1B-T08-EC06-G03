---
title: Banco de Dados
sidebar_position: 1
---
# Banco de Dados

## Introdução

Na seção [Visualização de Dados](/Sprint-3/visualizacao-de-dados.md), foram apresentados possíveis casos de uso dos dados que serão coletados pelo projeto. Nesta seção, haverá o detalhamento como esses dados estão armazenados no **banco de dados relacional**, conforme ilustrado abaixo:

<div align="center">

**Banco de dados Relacional**

![Banco de dados](/img/banco-de-dados-v2.png)


**Fonte:** Elaborado pela equipe Rebólins

</div>

A escolha de um banco de dados relacional foi feita devido à natureza estruturada dos dados do projeto. Essa escolha permite a realização de consultas complexas e filtragens eficientes, facilitadas pela linguagem SQL. Além disso, a integridade e a consistência dos dados são mantidas através das relações entre as tabelas, garantindo um armazenamento robusto e confiável.

## Descrição das tabelas

Abaixo está uma breve descrição de cada tabela, além de explicar seus campos juntamente com a razão dos mesmos.

### Order

A entidade **Order** representa um processo de limpeza que será realizado no reboiler. Sendo assim, ela possui a entidade filha **Examinations**, a qual contém as duas examinações que serão realizadas pelo robô, antes e após a limpeza. Ademais, a tabela **Order** têm campos como **reboiler_id**  e **robot_id**, os quais identificam o reboiler que estamos tratando e o robô específico que está realizando a limpeza. 

- **id**: Identificador único da verificação. (Chave primária)
- **status**: Diz se ainda está havendo uma verificação ou se foi concluída (text).
- **robot_id**: Referência para o robô que realizou a verificação. (chave estrangeira da tabela Robot, Integer)
- **reboiler_id**: Referência para o reboiler que está sendo verificado. (chave estrangeira da tabela Reboiler, Integer)
- **started_at**: Número inteiro indicando quando a verificação começou. (Integer)
- **finished_at**: Número inteiro indicando quando a verificação terminou. (Integer)

### Examination

A entidade **Examination** funciona como uma sessão de verificação do reboiler. Sendo assim, ela registra uma examinação realizada no reboiler, armazenando caracterísitcas como a etapa em que foi realizada (pré ou pós-limpeza), assim como o tempo que a sessão durou.

- **id**: Identificador único da verificação. (Chave primária)
- **etapa**: Etapa da verificação (text).
- **started_at**: Número inteiro indicando quando a verificação começou. (text)
- **finished_at**: Número inteiro indicando quando a verificação terminou. (text)
- **order_id**: Referência a tabela Order para indicar a qual processo ela está atrelada. (chave estrangeira da tabela Order, Integer)

### Image

A entidade **Image** tem como papel principal representar uma imagem de um determinado tubo, capturado durante uma das examinações. A criação da entidade permite com que se tenha uma base de diversas imagens, o que serviria para alimentar o modelo de treinamento, ao identificar o que há e não há sujeira. 

- **id**: Identificador único da imagem. (Chave primária, Integer)
- **image**: String das imagens comprimidas dos tubos. (text)
- **taken_at**: Número inteiro indicando quando a imagem foi tirada. (integer)


### Robot

A entidade **Robot** armazena as informações de um determinado robô, contendo informações sua identificação. Sua necessidade surge para a aplicação poder se torna escalável, tendo controle de vários operações de maneira simultânea através da plataforma web.

- **id**: Identificador único do robô. (Chave primária, Integer)
- **nickname**: Nome dado ao robô como forma de identificá-lo. (text)
- **unit_id**: Identificador da unidade à qual o robô pertence. (Chave estrangeira, integer).

### TubeState

A entidade **TubeState** permite armazenar o estado de cada tubo de uma determinada examinação, ou seja, armazenar o estado de sujeira de um determinado tubo de um reboiler. A tabela possui uma relação de 1:1 com a tabela imagem, possibilitando rever a imagem tirada do tubo e analisando a sua sujeira.

- **id**: Identificador único do estado do tubo. (Chave primária, Integer)
- **dirtness**: Booleano indicando se o tubo está sujo. (bool)
- **image_id**: Referência para a imagem associada ao estado do tubo. (chave estrangeira da tabela Image)
- **examination_id**: Referência para a verificação associada ao estado do tubo. (chave estrangeirada tabela Examination)

### Reboiler

A entidade **Reboiler** tem como utilidade tornar único cada reboiler de uma determinada unidade. Com isso, seria possível identificar se existe algum maquinário que está havendo problemas mais frequentes ou que não está sendo limpo com tanta eficiência. 

- **id**: Identificador único do reboiler. (Chave primária)
- **number**: Número do reboiler. (integer)
- **unit_id**: Referência para a unidade à qual o reboiler pertence. (chave estrangeira da tabela Unit)

### Unit

A entidade **Unit** armazena informações sobre a unidade onde o robô operará, como estado e cidade. Com isso, é possível filtrar as limpezas por cidade, avaliando e comparando, por exemplo: o estado dos reboilers, a eficiência da limpeza e a frequência de limpezas realizadas em cada unidade.

- **id**: Identificador único da unidade. (Chave primária)
- **city**: Cidade onde a unidade está localizada. (text)
- **state**: Estado onde a unidade está localizada. (text)

## Relacionamentos

**Examination e Order**
Examination.order_Id refere-se a Order.id. Isso cria uma relação dois-para-um *(dois examinations podem ser realizadas por um robô)*. Isso se deve a uma regra de negócio em que um order tem uma examinação de pré-limpeza e pós-limpeza.

**Examination e TubeState**

TubeState.examination_id refere-se a Examination.id. Isso cria uma relação muitos-para-um *(muitos tubeState pertencem a uma examinação)*.

**Order e Robot**

Order.robot_id refere-se a Robot.id. Isso cria uma relação muitos-para-um *(muitos processos podem ser realizadas por um robô)*.

**Order e Reboiler**

Order.reboiler_id refere-se a Reboiler.id. Isso cria uma relação muitos-para-um *(muitas processos podem ser associadas a um reboiler)*.

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