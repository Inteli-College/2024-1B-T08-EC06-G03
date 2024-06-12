require('dotenv').config();

const app = require('./config/express')();
const config = require('config');
const port = config.get('server.port');

app.listen(port, () => {
    console.log(`Servidor rodando na porta ${port}`)
});
