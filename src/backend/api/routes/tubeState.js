const express = require('express');
const router = express.Router();
const tubeStateController = require('../controllers/tubeState');

router.get('/tubes', tubeStateController.getAllTubeStates);
router.get('/tubes/:id', tubeStateController.getTubeStateById);
router.post('/tubes', tubeStateController.createTubeState);
router.put('/tubes/:id', tubeStateController.updateTubeState);
router.delete('/tubes/:id', tubeStateController.deleteTubeState);
 
module.exports = router;
