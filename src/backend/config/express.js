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
const examinationRoutes = require('../api/routes/examination');
const tubeStateRoutes = require('../api/routes/tubeState');
const orderRoutes = require('../api/routes/order');


// Websockets Routes
const teleopRouter = require('../api/routes/teleop');
const cameraRouter = require('../api/routes/camera');

module.exports = () => {
    const app = express();
    app.use(helmet());
    app.use(cors(process.env.CORS_ORIGIN || config.get('server.cors')));
    app.use(morgan('dev'));
    
    app.set('port', process.env.PORT || config.get('server.port'));
    app.set('host', process.env.HOST || config.get('server.host'));
    app.use(json({limit: '50mb'}));

    // Api
    app.use('/api/robots', robotRoutes);
    app.use('/api/unities', unitRoutes);
    app.use('/api/reboilers', reboilerRoutes);
    app.use('/api/images', imageRoutes);
    app.use('/api/examinations', examinationRoutes);
    app.use('/api/tube-states', tubeStateRoutes);
    app.use('/api/orders', orderRoutes);

    // Websockets
    app.use('/teleop', teleopRouter);
    app.use('/camera', cameraRouter);

    return app;
};
