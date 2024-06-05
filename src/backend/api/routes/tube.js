const express = require('express');
const router = express.Router();
const robotController = require('../controllers/tube');

router.get('/tubes', robotController.getAllTubes);
router.get('/tubes/:id', robotController.getTubeById);
router.post('/tubes', robotController.createTube);
router.put('/tubes/:id', robotController.updateTube);
router.delete('/tubes/:id', robotController.deleteTube);
 
module.exports = router;
