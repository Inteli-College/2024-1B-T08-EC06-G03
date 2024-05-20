const express = require('express');
const json = require('body-parser').json;
const config = require('config');
const cors = require('cors');
const helmet = require('helmet');
const morgan = require('morgan');

const teleopRouter = require('../api/routes/teleop');

module.exports = () => {
    const app = express();
    app.use(helmet());
    app.use(cors(process.env.CORS_ORIGIN || config.get('server.cors')));
    app.use(morgan('dev'));

    app.set('port', process.env.PORT || config.get('server.port'));
    app.set('host', process.env.HOST || config.get('server.host'));
    app.use(json());

    app.use('/teleop', teleopRouter);

    return app;
};
