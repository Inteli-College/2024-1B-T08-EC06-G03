const express = require('express');
const router = express.Router();
const tubeStateController = require('../controllers/tubeState');

router.get('/', tubeStateController.getAllTubeStates);
router.get('/:id', tubeStateController.getTubeStateById);
router.post('/', tubeStateController.createTubeState);
router.put('/:id', tubeStateController.updateTubeState);
router.delete('/:id', tubeStateController.deleteTubeState);
 
module.exports = router;
