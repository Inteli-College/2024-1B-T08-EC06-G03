const express = require('express');
const router = express.Router();
const tubeController = require('../controllers/tube');

router.get('/tubes', tubeController.getAllTubes);
router.get('/tubes/:id', tubeController.getTubeById);
router.post('/tubes', tubeController.createTube);
router.put('/tubes/:id', tubeController.updateTube);
router.delete('/tubes/:id', tubeController.deleteTube);
 
module.exports = router;
