const Router = require('express').Router;
const json = require('body-parser').json;
const lidarController = require('../controllers/lidar');

const router = Router();

router.use(json());

router.post('/start', lidarController.startLidarWS);

module.exports = router;
