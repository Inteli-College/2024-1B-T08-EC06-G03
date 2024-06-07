const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robot');

router.get('/robots', robotController.getAllRobots);
router.get('/robots/:id', robotController.getRobotById);
router.post('/robots', robotController.createRobot);
router.put('/robots/:id', robotController.updateRobot);
router.delete('/robots/:id', robotController.deleteRobot);
 
module.exports = router;
