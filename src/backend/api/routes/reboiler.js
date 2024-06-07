const express = require('express');
const router = express.Router();
const reboilerController = require('../controllers/reboiler');

router.get('/reboiler', reboilerController.getAllReboilers);
router.get('/reboiler/:id', reboilerController.getReboilerById);
router.post('/reboiler', reboilerController.createReboiler);
router.put('/reboiler/:id', reboilerController.updateReboiler);
router.delete('/reboiler/:id', reboilerController.deleteReboiler);
 
module.exports = router;
