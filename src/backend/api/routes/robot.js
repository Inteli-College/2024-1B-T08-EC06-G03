const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robot');

router.get('/', robotController.getAllRobots);
router.get('/:id', robotController.getRobotById);
router.get('/unit/:id', robotController.getRobotByUnitId);
router.post('/', robotController.createRobot);
router.put('/:id', robotController.updateRobot);
router.delete('/:id', robotController.deleteRobot);
 
module.exports = router;
