const express = require('express');
const router = express.Router();
const tubeController = require('../controllers/tube');

router.get('/', tubeController.getAllTubes);
router.get('/:id', tubeController.getTubeById);
router.post('/', tubeController.createTube);
router.put('/:id', tubeController.updateTube);
router.delete('/:id', tubeController.deleteTube);
 
module.exports = router;
