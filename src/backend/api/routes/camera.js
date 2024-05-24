const Router = require('express').Router;
const json = require('body-parser').json;
const cameraController = require('../controllers/camera');

const router = Router();

router.use(json());

router.post('/start', cameraController.startCameraWS);

module.exports = router;
