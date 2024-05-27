const Router = require('express').Router;
const json = require('body-parser').json;
const teleopController = require('../controllers/teleop');

const router = Router();

router.use(json());

router.post('/start', teleopController.startTeleopWS);

module.exports = router;
