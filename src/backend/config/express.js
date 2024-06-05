const express = require('express');
const json = require('body-parser').json;
const config = require('config');
const cors = require('cors');
const helmet = require('helmet');
const morgan = require('morgan');

// Api Routes
const robotRoutes = require('../api/routes/robot');
const unityRoutes = require('../api/routes/unity');
const reboilerRoutes = require('../api/routes/reboiler');
const imageRoutes = require('../api/routes/image');
const tubeRoutes = require('../api/routes/tube');
const examinationRoutes = require('../api/routes/examination');
const tubeStateRoutes = require('../api/routes/tubeState'); 


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
    app.use(json());

    // Api
    app.use('/api/robots', robotRoutes);
    app.use('/api/unities', unityRoutes);
    app.use('/api/reboilers', reboilerRoutes);
    app.use('/api/images', imageRoutes);
    app.use('/api/tubes', tubeRoutes);
    app.use('/api/examinations', examinationRoutes);
    app.use('/api/tube-states', tubeStateRoutes);
    // Websockets
    app.use('/teleop', teleopRouter);
    app.use('/camera', cameraRouter);

    return app;
};
