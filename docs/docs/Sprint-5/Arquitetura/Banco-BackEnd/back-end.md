---
title: Back-End - API
sidebar_position: 2
---

O objetivo principal dessa página é documentar a criação e implementação de uma API realizada pelo grupo, visando interagir com o Banco de Dados existente. Para uma implementação eficaz da API, foram utilizadas tecnologias que estão em alta no mercado e que coincidiam com os objetivos do grupo: manter a de manter a aplicação com uma manutenibilidade fácil para os desenvolvedores, porém garantindo a eficiência em realizar consultas ao banco. Com isso, foi desenvolvida uma aplicação **express**, juntamente aos **websockets** já existentes.

## Tecnologias Utilizadas

- **JavaScript**
  - **[Express.js](https://expressjs.com/)**: Framework para construção de aplicações web, utilizado para criar o servidor HTTP.
- **Turso**
  - [**turso**:](https://turso.tech/) Banco de dados em nuvem, o qual funciona rodando um database do tipo sqlite. Para compreender mais sobre a decisão de utilizar turso, veja [aqui](/Sprint-5/Arquitetura/Banco-BackEnd/banco-de-dados.md).
- **Prisma ORM**
  - [**prisma.io**](https://www.prisma.io/docs/orm/prisma-schema/overview): Biblioteca utilizada para servir como intermédio entre a api e o banco de dados, realizando as consultas por ORM. As consultas por ORM garantem mais segurança para a aplicação, visto que evitam problemas de cybersegurança como **sqlinjection**. Além disso, a rapidez para realizar seu desenvolvimento auxilia muito no projeto, visto o escopo de dez semanas.


## Arquitetura

Antes de iniciar o desenvolvimento do back-end da aplicação,  foi idealizado e implementado o banco de dados, o qual é explicado em [outra página](/Sprint-5/Arquitetura/Banco-BackEnd/banco-de-dados.md). Sua implementação em **Turso**, o qual tem interatividade com o **Prisma.Io**, permite com que a aplicação seja desenvolvida de forma mais acelerada devido a funções já existentes para realizar as operações de leitura, inserção, atualização dos dados.

O back-end foi construído por meio de arquivos de controllers e de routes, importando os routers para a aplicação principal que funciona em express. Sendo assim, cada entidade possui dois arquivos, o controller, o qual possui o crud da entidade e outras operações que são executadas nela, e o route, o qual utiliza das funções implementadas no controller.

Aqui podemos ver o arquivo `express.js`, o qual é responsável por fazer a api funcionar na totalidade.

```javascript
const express = require('express');
const json = require('body-parser').json;
const config = require('config');
const cors = require('cors');
const helmet = require('helmet');
const morgan = require('morgan');

// Api Routes
const robotRoutes = require('../api/routes/robot');
const unitRoutes = require('../api/routes/unit');
const reboilerRoutes = require('../api/routes/reboiler');
const imageRoutes = require('../api/routes/image');
const orderRoutes = require('../api/routes/order');
const examinationRoutes = require('../api/routes/examination');
const tubeStateRoutes = require('../api/routes/tubeState'); 

module.exports = () => {
    const app = express();
    app.use(helmet());
    app.use(cors(process.env.CORS_ORIGIN || config.get('server.cors')));
    app.use(morgan('dev'));
    
    app.set('port', process.env.PORT || config.get('server.port'));
    app.set('host', process.env.HOST || config.get('server.host'));
    app.use(json());

    // Definição do nome de cada rota da api.
    app.use('/api/robots', robotRoutes);
    app.use('/api/unities', unitRoutes);
    app.use('/api/reboilers', reboilerRoutes);
    app.use('/api/images', imageRoutes);
    app.use('/api/orders', tubeRoutes);
    app.use('/api/examinations', examinationRoutes);
    app.use('/api/tube-states', tubeStateRoutes);

    return app;
};
```

# Metodologia e Implementação

## Implementação 

### Criando o database
É necessário criar uma conta na aplicação do [**Turso**](https://turso.tech/) e seguir o [tutorial](https://docs.turso.tech/quickstart), até o passo 4, para configurar o terminal. Após criar a conta e conectar o banco de dados ao terminal, basta executar na plataforma os comandos sql presente no diretório:
```bash
/src/backend/scripts/createDatabase.sql
```

### Conectando o database na aplicação
Após criado a base de dados, é necessário seguir um [segundo tutorial](https://docs.turso.tech/sdk/ts/orm/prisma), o qual permitirá realizar a conexão entre o **Turso** e o **Prisma** em sua máquina. Os comandos devem ser rodados no diretório do `backend`. Nesse tutorial, será necessário criar um arquivo chamado `schema.prisma`, o qual ficará localizado em: 
```bash
/src/backend/models/
```
Ele deve conter a modelagem do banco, porém transformada em arquivo do tipo .schema. 

:::warning Aviso
Como o arquivo não está na pasta raiz do backend, o comando `npx prisma generate` deve ser substituído por `npx prisma generate --schema=api/models/schema.prisma`.
:::

Em sequência, é necessário criar um arquivo .env na pasta `backend`. Esse arquivo armazenará os tokens para conectar ao turso.

```env
TURSO_DATABASE_URL={url}
TURSO_AUTH_TOKEN={token}
```

É necessário substituir as variáveis pelas informações geradas no tutorial.

## Metodologia

Na pasta models, deve-se criar um arquivo chamado de `prismaclient.js`. Ele será responsável por inicializar a conexão entre o prisma e o banco de dados, além disso, ele deve ser importado em todo controller criado para poder realizar as chamadas necessárias. O código dele será dessa forma: 
```javascript
const { PrismaClient } = require('@prisma/client')
const { createClient } = require('@libsql/client');
const { PrismaLibSQL } = require('@prisma/adapter-libsql');

const libsql = createClient({
  url: process.env.TURSO_DATABASE_URL,
  authToken: process.env.TURSO_AUTH_TOKEN,
});

const adapter = new PrismaLibSQL(libsql);
const prisma = new PrismaClient({ adapter });

module.exports = prisma;
``` 

### Criando um controller

Para criar um controller, basta importar o `prisma` que criamos no último trecho de código para seu controller e utilizá-lo para fazer requisições. Um exemplo seria:
```javascript
const prisma = require('../models/prismaClient');

const getAllRobots = async (req, res) => {
  try {
    const robots = await prisma.robot.findMany();
    res.json(robots);
  } catch (error) {
    res.status(500).json({ error: 'Error fetching robots' });
  }
};
```
## Rotas 

Para a documentação das rotas, foi utilizado o Postman, o qual contém recursos que facilitam a demonstração das mesmas. [Link de acesso](https://www.postman.com/planetary-astronaut-106586/workspace/reboilns-g03/documentation/26958099-16b60531-2533-4099-a7a3-ba5241ad8537)