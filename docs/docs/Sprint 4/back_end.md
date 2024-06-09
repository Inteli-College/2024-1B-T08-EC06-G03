---
title: Back-End - API
sidebar_position: 2
---

O objetivo principal dessa página é documentar a criação e implementação de uma API realizada pelo grupo, com o objetivo de interagir com o Banco de Dados existente. Para uma implementação eficaz da API, foram utilizadas tecnologias que estão em alta no mercado e que coiencidiam com os objetivos do grupo, de manter a aplicação com uma manutenibilidade fácil para os desenvolvedores, porém ainda tendo eficiência em realizar consultas ao banco. Com isso, foi desenvolvida uma aplicação **express**, juntamente aos **websockets** já existentes.

## Tecnologias Utilizadas

- **JavaScript**
  - **[Express.js](https://expressjs.com/)**: Framework para construção de aplicações web, utilizado para criar o servidor HTTP.
- **Turso**
  - [**turso**:](https://turso.tech/) Banco de dados em nuvem, o qual funciona rodando um database do tipo sqlite.
- **Prisma ORM**
  - [**prisma.io**](https://www.prisma.io/docs/orm/prisma-schema/overview): Biblioteca utilziada para servir como intermédio entre a api e o banco de dados, realizando as consultas através de ORM.


## Arquitetura

Para que fosse possível desenvolver o back-end da aplicação, primeiro foi idealizado o banco de dados, o qual é explicado em [outra página](/Sprint%204/banco_de_dados.md). Sua implementação em **Turso**, o qual tem interatividade com o **Prisma.Io**, permite com que a aplicação seja desenvolvida de forma mais acelerada devido a funções já existentes para realizar as operações de leitura, inserção, atualização dos dados.



# Metodologia e Implementação

### Criando o database
É necessário criar uma conta na aplicação do [**Turso**](https://turso.tech/) e seguir o [tutorial](https://docs.turso.tech/quickstart), até o passo 4, para configurar o terminal. Após criar a conta e conectar o banco de dados ao terminal, basta executar os comandos sql presente no diretório:
```bash
/src/backend/scripts/createDatabase.sql
```
 na plataforma para criar sua base de dados.

### Conectando o database na aplicação
Depois de criado a base de dados, é necessário seguir um [segundo tutorial](https://docs.turso.tech/sdk/ts/orm/prisma), o qual permitirá realizar a conexão entre o **Turso** e o **Prisma** em sua máquina. Os comandos devem ser rodados no diretório do `backend`. Nesse tutorial, será necessário criar um arquivo chamado `prisma.schema`, o qual ficará localizado em: 
```bash
/src/backend/models/
```
Ele deve conter a modelagem do banco, porém transformada em arquivo do tipo .schema. 

::: warning [Aviso]
Como o arquivo não está na pasta raiz do backend, o comando `npx generate schema` deve ser substituído por `npx generate schmea --schema=api/models/prisma.schema`.
:::
